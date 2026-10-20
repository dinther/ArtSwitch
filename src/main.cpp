// ArtSwitch - Art-Net controlled relays on a WT32-ETH01
//
// - Ethernet with DHCP; falls back to a self-assigned 169.254.x.x address (from the MAC) if no
//   DHCP server answers. A server that shows up within 2 minutes still wins; after that the
//   address is held until the cable is re-seated or the board restarts (as in tapbox)
// - Reachable as http://<name>.local (mDNS), default http://artswitch.local
// - Answers ArtPoll, so it shows up in consoles / DMX-Workshop / QLC+
// - Up to 3 relays on consecutive DMX channels from a base address; pins set on the
//   web page (active-high, off at boot)
// - Relays hold their last state when Art-Net stops
// - Firmware updates over Ethernet (ArduinoOTA, see platformio.ini env:ota)
// - WiFi as a backup: joins the WiFi network set on the web page while the Ethernet cable is
//   unplugged, and opens an open hotspot named after the board when that network can't be
//   reached (or none is set). WiFi is switched off as soon as the cable is back.

#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <lwip/sockets.h>
#include <Preferences.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_netif.h>

// Every board starts with this name; saving or restarting is refused until it is changed,
// so two boards on one network don't both answer to artswitch.local
static const char *DEFAULT_NAME = "artswitch";
static const int RELAY_COUNT = 3;
static const int8_t NO_PIN = -1;
// Onboard green LED4 sits on IO17 (TXD2) and lights when the pin is low (datasheet V1.4 4.2,
// verified on the board). It mirrors relay 1's output for testing.
static const int RELAY1_LED_PIN = 17;
static const uint16_t ARTNET_PORT = 6454;
static const uint16_t SACN_PORT = 5568;
static const uint16_t SACN_MAX_UNIVERSE = 63999;
// A source that sends nothing for this long is gone (E1.31 network data loss timeout)
static const uint32_t SACN_SOURCE_TIMEOUT_MS = 2500;

enum Protocol : uint8_t { PROTO_ARTNET, PROTO_SACN, PROTO_BOTH };
const char *const PROTOCOL_NAMES[] = {"Art-Net", "sACN", "Art-Net and sACN"};
static const uint32_t DHCP_TIMEOUT_MS = 10000;
static const uint32_t DHCP_PROBE_INTERVAL_MS = 15000;
// Once someone may have configured against the fallback address, it must not move mid-show
static const uint32_t DHCP_GRACE_MS = 120000;
// WiFi waits this long without an Ethernet link, so a link coming up at boot or a cable being
// re-seated doesn't start WiFi
static const uint32_t WIFI_START_DELAY_MS = 5000;
static const uint32_t WIFI_JOIN_TIMEOUT_MS = 30000;  // then open the hotspot
static const uint32_t WIFI_RETRY_MS = 10000;         // between join attempts
// While the hotspot is up, joining is retried only when nobody is connected to it: a join
// attempt scans all channels and would drop the phone using the hotspot
static const uint32_t HOTSPOT_RETRY_MS = 30000;
static const uint32_t SCAN_TIMEOUT_MS = 15000;

// Default hysteresis around the 50% point so a fader parked mid-way doesn't chatter a relay
static const uint8_t DEFAULT_ON_AT = 140;
static const uint8_t DEFAULT_OFF_AT = 115;
static const uint16_t MAX_MIN_SWITCH_MS = 60000;

static const uint16_t OP_POLL = 0x2000;
static const uint16_t OP_POLL_REPLY = 0x2100;
static const uint16_t OP_DMX = 0x5000;

// GPIOs on the WT32-ETH01 headers that are free for outputs. Ethernet uses 0, 16, 18, 19,
// 21-23, 25-27; 1/3 are the programming UART; 35/36/39 are input-only; 12 is left out because
// a pull-up on it at boot selects 1.8 V flash and the board won't start.
// The V1.4 silkscreen labels some GPIOs by function; other revisions may differ, so the
// GPIO number always comes first.
struct PinOption {
  int8_t pin;
  const char *name;
  const char *note;
};
// IO17 is not offered: it drives onboard LED4, used as the relay 1 indicator.
static const PinOption PIN_OPTIONS[] = {
    {4, "IO4", ""},
    {32, "IO32 (CFG)", ""},
    {33, "IO33 (485_EN)", ""},
    {2, "IO2", " - must not be pulled high while flashing"},
    {5, "IO5 (RXD)", " - may pulse at boot"},
    {14, "IO14", " - may pulse at boot"},
    {15, "IO15", " - may pulse at boot"},
};

bool isAllowedPin(int pin) {
  if (pin == NO_PIN) return true;
  for (const PinOption &o : PIN_OPTIONS)
    if (o.pin == pin) return true;
  return false;
}

const char *pinName(int pin) {
  for (const PinOption &o : PIN_OPTIONS)
    if (o.pin == pin) return o.name;
  return "";
}

struct RelayConfig {
  int8_t pin;                 // GPIO, NO_PIN if unused
  uint8_t onAt;               // switch on when DMX value >= onAt
  uint8_t offAt;              // switch off when DMX value <= offAt (offAt < onAt)
  uint16_t minSwitchMs;       // protects contacts/loads from fast chases; 0 = no limit
};

struct Settings {
  String name;                // mDNS hostname and Art-Net short name
  uint8_t protocol;           // Protocol: which of Art-Net / sACN switch the relays
  uint16_t universe;          // 15-bit Art-Net port-address (Net:SubNet:Universe)
  uint16_t sacnUniverse;      // sACN universe, 1-63999
  uint16_t baseChannel;       // relay n listens on baseChannel + n
  RelayConfig relay[RELAY_COUNT];
  IPAddress fallbackIp;       // used when DHCP fails; 0.0.0.0 = automatic 169.254.x.x
  String wifiSsid;            // backup WiFi network, empty = hotspot only
  String wifiPass;            // empty = open network
  bool oscEnabled;
  uint16_t oscPort;
  IPAddress oscFeedbackIp;    // relay changes are sent here; 0.0.0.0 = no feedback
  uint16_t oscFeedbackPort;
};

struct Relay {
  int8_t pin = NO_PIN;
  volatile bool on = false;   // output state
  volatile bool want = false; // state asked for by DMX or test; held back by minSwitchMs
  bool switched = false;      // first switch after boot / pin change is never delayed
  uint32_t lastSwitchAt = 0;
  volatile int value = -1;    // last DMX value, -1 = none since boot / settings change
};

Settings settings;
Relay relays[RELAY_COUNT];
Preferences prefs;
WiFiUDP udp;
int sacnSocket = -1;  // plain socket: WiFiUDP can join only one multicast group
WiFiUDP oscUdp;
uint16_t oscOpenPort = 0;  // port oscUdp listens on, 0 = closed
AsyncWebServer server(80);

// Web handlers run in the async TCP task; they hand changes to loop() through these
Settings pendingSettings;
volatile bool settingsPending = false;
volatile bool restartRequested = false;
volatile int8_t pendingTest[RELAY_COUNT] = {-1, -1, -1};  // -1 none, 0 off, 1 on

volatile bool linkUp = false;
volatile bool gotIp = false;
uint32_t linkUpAt = 0;
volatile uint32_t linkDownAt = 0;  // boot counts as link down
bool usingFallback = false;
bool networkServicesStarted = false;
String runningName;  // hostname mDNS was started with; a saved new name applies after restart
volatile bool dhcpRestartRequested = false;

// During the grace window on the fallback address, a DHCPDISCOVER is sent now and then; an
// offer means a server has appeared (e.g. the router booted after us) and we switch to DHCP.
WiFiUDP dhcpProbe;
bool dhcpProbing = false;
uint32_t dhcpProbeXid = 0;
uint32_t fallbackSince = 0;
uint32_t lastDhcpProbeAt = 0;

enum WifiState { WIFI_STANDBY, WIFI_JOINING, WIFI_JOINED, WIFI_HOTSPOT };
volatile WifiState wifiState = WIFI_STANDBY;
volatile bool wifiGotIp = false;
uint32_t wifiStateAt = 0;
uint32_t lastJoinAt = 0;
// On the hotspot there is no router to resolve names, so the board answers every name with its
// own address: http://<name> works, and a phone's internet check lands on the settings page
DNSServer hotspotDns;

// WiFi network scan for the settings page. The web handler only asks; loop() runs the scan and
// fills scanResults (JSON) before setting SCAN_DONE, so the handler never reads it half-written.
enum ScanState { SCAN_IDLE, SCAN_REQUESTED, SCAN_RUNNING, SCAN_DONE, SCAN_FAILED };
volatile ScanState scanState = SCAN_IDLE;
String scanResults;
bool scanRadioOnly = false;
uint32_t scanStartedAt = 0;  // radio switched on just for the scan, off again afterwards

uint32_t lastDmxAt = 0;
uint32_t dmxPackets = 0;
uint32_t pollReplies = 0;
uint32_t lastPollFrom = 0;  // shown on the web page to check controller discovery
uint32_t lastPollAt = 0;

// Recently seen Art-Net and sACN DMX sources, any universe - shown on the web page for troubleshooting
struct TrafficSeen {
  uint32_t ip;
  uint16_t universe;
  bool sacn;
  char name[33];              // sACN source name (Art-Net DMX packets carry none)
  uint8_t priority;           // sACN priority
  uint32_t lastAt;
};
TrafficSeen traffic[8];
int trafficCount = 0;

// sACN source names are up to 64 bytes and not always terminated; 32 characters is plenty here
void copySacnName(char *to, const uint8_t *from) {
  int n = 0;
  while (n < 32 && from[n]) to[n] = from[n], n++;
  to[n] = 0;
}

void noteTraffic(IPAddress ip, uint16_t universe, bool sacn, const uint8_t *name = nullptr, uint8_t priority = 0) {
  int slot = -1;
  for (int i = 0; i < trafficCount; i++)
    if (traffic[i].ip == (uint32_t)ip && traffic[i].universe == universe && traffic[i].sacn == sacn) slot = i;
  if (slot < 0) {
    if (trafficCount < 8) {
      slot = trafficCount++;
    } else {  // replace the stalest entry
      slot = 0;
      for (int i = 1; i < 8; i++)
        if (traffic[i].lastAt < traffic[slot].lastAt) slot = i;
    }
    traffic[slot].ip = ip;
    traffic[slot].universe = universe;
    traffic[slot].sacn = sacn;
  }
  if (name) copySacnName(traffic[slot].name, name);
  else traffic[slot].name[0] = 0;
  traffic[slot].priority = priority;
  traffic[slot].lastAt = millis();
}

// sACN Universe Discovery (E1.31 section 8): sources announce the universes they send every
// 10 s. Shown on the web page to help find which universe a controller uses.
static const uint16_t SACN_DISCOVERY_UNIVERSE = 64214;
static const uint32_t SACN_ANNOUNCE_TIMEOUT_MS = 30000;
struct SacnAnnounce {
  uint8_t cid[16];
  uint32_t ip;
  char name[33];
  uint16_t universes[16];     // the first 16 announced
  uint16_t count;             // all announced, may be more than 16
  uint32_t lastAt;
};
SacnAnnounce announces[4];
int announceCount = 0;

// sACN sources sending to our universe; the highest priority one is followed
struct SacnSource {
  uint8_t cid[16];
  uint8_t priority;
  uint8_t sequence;
  uint32_t lastAt;
};
SacnSource sacnSources[4];
int sacnSourceCount = 0;
// Universe and interfaces the sACN socket joined; rejoined when any of them changes
String sacnJoinedKey;

uint8_t packet[640];  // largest: sACN with 512 slots, 638 bytes

// ---------------------------------------------------------------- settings

String sanitizeName(const String &in) {
  String out;
  for (char c : in) {
    c = tolower(c);
    if (isalnum(c) || c == '-') out += c;
    if (out.length() >= 31) break;
  }
  // Hostnames can't start or end with '-'
  while (out.startsWith("-")) out.remove(0, 1);
  while (out.endsWith("-")) out.remove(out.length() - 1);
  return out.length() ? out : String(DEFAULT_NAME);
}

// WiFi network names can contain any character
String htmlEscape(const String &in) {
  String out;
  for (char c : in) {
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

String jsonEscape(const String &in) {
  String out;
  for (char c : in) {
    if (c == '"' || c == '\\') out += '\\';
    if ((uint8_t)c >= 0x20) out += c;
  }
  return out;
}

// Link-local address from the MAC, so several boards without DHCP don't collide and a PC
// that self-assigned 169.254.x.x is on the same subnet. 169.254.0.x and .255.x are reserved.
IPAddress autoFallbackIp() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_ETH);
  return IPAddress(169, 254, 1 + mac[4] % 254, mac[5]);
}

IPAddress fallbackIp() {
  return (uint32_t)settings.fallbackIp ? settings.fallbackIp : autoFallbackIp();
}

// Classful netmask: 2.x / 10.x get /8, 169.254.x gets /16, 192.168.x gets /24
IPAddress netmaskFor(IPAddress ip) {
  if (ip[0] < 128) return IPAddress(255, 0, 0, 0);
  if (ip[0] < 192) return IPAddress(255, 255, 0, 0);
  return IPAddress(255, 255, 255, 0);
}

void loadSettings() {
  prefs.begin("artswitch", true);
  settings.name = prefs.getString("name", DEFAULT_NAME);
  settings.protocol = min((int)prefs.getUChar("proto", PROTO_ARTNET), (int)PROTO_BOTH);
  settings.universe = prefs.getUShort("universe", 0);
  settings.sacnUniverse = constrain(prefs.getUShort("sacnuni", 1), 1, SACN_MAX_UNIVERSE);
  settings.baseChannel = prefs.getUShort("channel", 1);
  for (int i = 0; i < RELAY_COUNT; i++) {
    RelayConfig &c = settings.relay[i];
    char key[6];
    snprintf(key, sizeof(key), "pin%d", i + 1);
    int8_t pin = prefs.getChar(key, i == 0 ? 4 : NO_PIN);
    c.pin = isAllowedPin(pin) ? pin : NO_PIN;
    snprintf(key, sizeof(key), "on%d", i + 1);
    c.onAt = prefs.getUChar(key, DEFAULT_ON_AT);
    snprintf(key, sizeof(key), "off%d", i + 1);
    c.offAt = prefs.getUChar(key, DEFAULT_OFF_AT);
    if (c.offAt >= c.onAt) {
      c.onAt = DEFAULT_ON_AT;
      c.offAt = DEFAULT_OFF_AT;
    }
    snprintf(key, sizeof(key), "min%d", i + 1);
    c.minSwitchMs = min(prefs.getUShort(key, 0), MAX_MIN_SWITCH_MS);
  }
  settings.fallbackIp = IPAddress(prefs.getUInt("fallback", 0));
  settings.wifiSsid = prefs.getString("wssid", "");
  settings.wifiPass = prefs.getString("wpass", "");
  settings.oscEnabled = prefs.getBool("osc", false);
  settings.oscPort = prefs.getUShort("oscport", 8000);
  settings.oscFeedbackIp = IPAddress(prefs.getUInt("oscfbip", 0));
  settings.oscFeedbackPort = prefs.getUShort("oscfbport", 9000);
  prefs.end();
}

void saveSettings() {
  prefs.begin("artswitch", false);
  prefs.putString("name", settings.name);
  prefs.putUChar("proto", settings.protocol);
  prefs.putUShort("universe", settings.universe);
  prefs.putUShort("sacnuni", settings.sacnUniverse);
  prefs.putUShort("channel", settings.baseChannel);
  for (int i = 0; i < RELAY_COUNT; i++) {
    const RelayConfig &c = settings.relay[i];
    char key[6];
    snprintf(key, sizeof(key), "pin%d", i + 1);
    prefs.putChar(key, c.pin);
    snprintf(key, sizeof(key), "on%d", i + 1);
    prefs.putUChar(key, c.onAt);
    snprintf(key, sizeof(key), "off%d", i + 1);
    prefs.putUChar(key, c.offAt);
    snprintf(key, sizeof(key), "min%d", i + 1);
    prefs.putUShort(key, c.minSwitchMs);
  }
  prefs.putUInt("fallback", (uint32_t)settings.fallbackIp);
  prefs.putString("wssid", settings.wifiSsid);
  prefs.putString("wpass", settings.wifiPass);
  prefs.putBool("osc", settings.oscEnabled);
  prefs.putUShort("oscport", settings.oscPort);
  prefs.putUInt("oscfbip", (uint32_t)settings.oscFeedbackIp);
  prefs.putUShort("oscfbport", settings.oscFeedbackPort);
  prefs.end();
}

// ---------------------------------------------------------------- relays

void sendOscFeedback(int i);

// Bring the output to the wanted state, but not sooner than minSwitchMs after the previous
// switch. A change asked for during the lockout is carried out when it ends, if still wanted.
void updateRelay(int i) {
  Relay &r = relays[i];
  if (r.want == r.on) return;
  if (r.switched && millis() - r.lastSwitchAt < settings.relay[i].minSwitchMs) return;
  r.on = r.want;
  r.switched = true;
  r.lastSwitchAt = millis();
  if (r.pin != NO_PIN) digitalWrite(r.pin, r.on ? HIGH : LOW);
  Serial.printf("Relay %d %s\n", i + 1, r.on ? "ON" : "OFF");
  sendOscFeedback(i);
}

void requestRelay(int i, bool on) {
  relays[i].want = on;
  updateRelay(i);
}

uint32_t lockoutRemainingMs(int i) {
  const Relay &r = relays[i];
  uint32_t minMs = settings.relay[i].minSwitchMs;
  if (r.want == r.on || !r.switched) return 0;
  uint32_t since = millis() - r.lastSwitchAt;
  return since < minMs ? minMs - since : 0;
}

// Switch a relay to another GPIO: the old pin is driven off and released, the new one starts off
void assignRelayPin(int i, int8_t pin) {
  Relay &r = relays[i];
  if (r.pin == pin) return;
  if (r.pin != NO_PIN) {
    digitalWrite(r.pin, LOW);
    pinMode(r.pin, INPUT);
  }
  r.pin = pin;
  r.on = r.want = r.switched = false;
  if (pin != NO_PIN) {
    // Drive low before enabling the output so the relay never clicks on
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  Serial.printf("Relay %d on %s\n", i + 1, pin == NO_PIN ? "no pin" : pinName(pin));
}

// Hysteresis works on the wanted state, so a held-back switch doesn't change the thresholds
void applyDmxValue(int i, uint8_t value) {
  const RelayConfig &c = settings.relay[i];
  relays[i].value = value;
  if (!relays[i].want && value >= c.onAt) requestRelay(i, true);
  else if (relays[i].want && value <= c.offAt) requestRelay(i, false);
}

// DMX slots from Art-Net or sACN for our universe; slot 0 is channel 1
void applyDmxFrame(const uint8_t *dmx, int count) {
  bool used = false;
  for (int i = 0; i < RELAY_COUNT; i++) {
    int index = settings.baseChannel + i - 1;
    if (index >= count) continue;
    applyDmxValue(i, dmx[index]);
    used = true;
  }
  if (used) {
    dmxPackets++;
    lastDmxAt = millis();
  }
}

void applyPendingTests() {
  for (int i = 0; i < RELAY_COUNT; i++) {
    int8_t t = pendingTest[i];
    if (t < 0) continue;
    pendingTest[i] = -1;
    requestRelay(i, t == 1);
  }
}

// ---------------------------------------------------------------- network

void onNetworkEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname(settings.name.c_str());
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet link up");
      linkUp = true;
      linkUpAt = millis();
      // Cable may now be in a network with DHCP
      if (usingFallback) dhcpRestartRequested = true;
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("IP address: %s (DHCP)\n", ETH.localIP().toString().c_str());
      gotIp = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet link down");
      linkUp = false;
      linkDownAt = millis();
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("WiFi IP address: %s\n", WiFi.localIP().toString().c_str());
      wifiGotIp = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      wifiGotIp = false;
      break;
    default:
      break;
  }
}

void checkDhcpFallback() {
  if (gotIp || usingFallback || !linkUp) return;
  if (millis() - linkUpAt < DHCP_TIMEOUT_MS) return;
  IPAddress ip = fallbackIp();
  ETH.config(ip, IPAddress(0, 0, 0, 0), netmaskFor(ip));
  usingFallback = true;
  gotIp = true;
  dhcpProbe.begin(68);
  dhcpProbing = true;
  fallbackSince = lastDhcpProbeAt = millis();
  Serial.printf("No DHCP answer, using fallback IP %s/%s\n", ip.toString().c_str(),
                netmaskFor(ip).toString().c_str());
}

void restartDhcp() {
  dhcpRestartRequested = false;
  if (!usingFallback) return;
  dhcpProbe.stop();
  dhcpProbing = false;
  ETH.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
  usingFallback = false;
  gotIp = false;
  linkUpAt = millis();  // fall back again if this attempt gets no lease either
  Serial.println("Retrying DHCP");
}

void sendDhcpDiscover() {
  uint8_t d[300] = {0};
  d[0] = 1;     // BOOTREQUEST
  d[1] = 1;     // Ethernet
  d[2] = 6;     // MAC length
  dhcpProbeXid = esp_random();
  memcpy(&d[4], &dhcpProbeXid, 4);
  d[10] = 0x80; // ask for a broadcast reply, we have no lease
  esp_read_mac(&d[28], ESP_MAC_ETH);
  const uint8_t options[] = {0x63, 0x82, 0x53, 0x63, 53, 1, 1, 255};  // magic cookie, DISCOVER, end
  memcpy(&d[236], options, sizeof(options));
  dhcpProbe.beginPacket(IPAddress(255, 255, 255, 255), 67);
  dhcpProbe.write(d, sizeof(d));
  dhcpProbe.endPacket();
}

void probeForDhcpServer() {
  if (!dhcpProbing) return;
  if (millis() - fallbackSince >= DHCP_GRACE_MS) {
    dhcpProbe.stop();
    dhcpProbing = false;
    Serial.println("No DHCP server within grace window, holding fallback address");
    return;
  }
  int len = dhcpProbe.parsePacket();
  if (len > 0) {
    uint8_t d[600];
    len = dhcpProbe.read(d, min(len, (int)sizeof(d)));
    dhcpProbe.flush();
    if (len >= 244 && d[0] == 2 && memcmp(&d[4], &dhcpProbeXid, 4) == 0) {
      // Walk the options for message type 53 = OFFER (2)
      for (int i = 240; i + 2 < len && d[i] != 255;) {
        if (d[i] == 0) { i++; continue; }
        if (d[i] == 53 && d[i + 2] == 2) {
          Serial.println("DHCP server found");
          restartDhcp();
          return;
        }
        i += 2 + d[i + 1];
      }
    }
  }
  if (millis() - lastDhcpProbeAt >= DHCP_PROBE_INTERVAL_MS) {
    lastDhcpProbeAt = millis();
    sendDhcpDiscover();
  }
}

// ---------------------------------------------------------------- WiFi backup

void setWifiState(WifiState s) {
  wifiState = s;
  wifiStateAt = millis();
}

void joinWifi() {
  lastJoinAt = millis();
  WiFi.begin(settings.wifiSsid.c_str(), settings.wifiPass.c_str());
}

// WiFi joins with the Ethernet MAC, so the router sees the same device and hands out the same
// address (and name) as on the cable. Only one of the two is ever active, so the MAC never
// appears twice on the network. Set on the driver and on the IP interface, which keeps its own
// copy for DHCP and ARP.
void useEthernetMacForWifi() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_ETH);
  esp_wifi_set_mac(WIFI_IF_STA, mac);
  esp_netif_set_mac(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), mac);
}

void startHotspot() {
  // Keep the station side when a network is set, so joining can be retried from the hotspot
  WiFi.mode(settings.wifiSsid.length() ? WIFI_AP_STA : WIFI_AP);
  if (settings.wifiSsid.length()) useEthernetMacForWifi();
  WiFi.softAP(runningName.c_str());
  hotspotDns.start(53, "*", WiFi.softAPIP());
  lastJoinAt = millis();
  setWifiState(WIFI_HOTSPOT);
  Serial.printf("WiFi hotspot \"%s\" open, http://%s\n", runningName.c_str(),
                WiFi.softAPIP().toString().c_str());
}

void startWifi() {
  WiFi.setHostname(runningName.c_str());
  WiFi.setAutoReconnect(false);  // retries are paced by updateWifi()
  if (!settings.wifiSsid.length()) {
    startHotspot();
    return;
  }
  WiFi.mode(WIFI_STA);
  useEthernetMacForWifi();
  joinWifi();
  setWifiState(WIFI_JOINING);
  Serial.printf("No Ethernet link, joining WiFi \"%s\"\n", settings.wifiSsid.c_str());
}

void stopWifi() {
  if (wifiState == WIFI_STANDBY) return;
  hotspotDns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiGotIp = false;
  setWifiState(WIFI_STANDBY);
  Serial.println("WiFi off");
}

// WiFi only runs while the Ethernet link is down, so the board is never on the network twice
void updateWifi() {
  if (linkUp) {
    stopWifi();
    return;
  }
  if (scanRadioOnly) return;  // let the scan finish before WiFi takes over the radio
  // The fallback-address DHCP probe belongs to Ethernet; a re-plugged cable retries DHCP anyway
  if (dhcpProbing) {
    dhcpProbe.stop();
    dhcpProbing = false;
  }
  uint32_t now = millis();
  switch (wifiState) {
    case WIFI_STANDBY:
      if (now - linkDownAt >= WIFI_START_DELAY_MS) startWifi();
      break;
    case WIFI_JOINING:
      if (wifiGotIp) {
        setWifiState(WIFI_JOINED);
        Serial.printf("Joined WiFi \"%s\"\n", settings.wifiSsid.c_str());
      } else if (now - wifiStateAt >= WIFI_JOIN_TIMEOUT_MS) {
        startHotspot();
      } else if (now - lastJoinAt >= WIFI_RETRY_MS) {
        joinWifi();
      }
      break;
    case WIFI_JOINED:
      if (!wifiGotIp) {
        Serial.println("WiFi connection lost");
        joinWifi();
        setWifiState(WIFI_JOINING);
      }
      break;
    case WIFI_HOTSPOT:
      if (wifiGotIp) {
        hotspotDns.stop();
        WiFi.softAPdisconnect(true);
        setWifiState(WIFI_JOINED);
        Serial.printf("Joined WiFi \"%s\", hotspot closed\n", settings.wifiSsid.c_str());
      } else if (settings.wifiSsid.length() && WiFi.softAPgetStationNum() == 0 &&
                 now - lastJoinAt >= HOTSPOT_RETRY_MS) {
        joinWifi();
      } else {
        hotspotDns.processNextRequest();
      }
      break;
  }
}

void updateScan() {
  if (scanState == SCAN_REQUESTED) {
    if (wifiState == WIFI_JOINING) return;  // can't scan during a join attempt; wait for it
    if (wifiState == WIFI_STANDBY) {
      WiFi.mode(WIFI_STA);
      scanRadioOnly = true;
    }
    scanStartedAt = millis();
    scanState = WiFi.scanNetworks(true) == WIFI_SCAN_FAILED ? SCAN_FAILED : SCAN_RUNNING;
    if (scanState == SCAN_RUNNING) return;
  } else if (scanState == SCAN_RUNNING) {
    // Some scans take longer than the 6 s after which WiFi.scanComplete() gives up (seen on the
    // board: done in 3 s or in over 6 s), so wait for the scan-done flag with our own limit
    bool done = WiFiGenericClass::getStatusBits() & WIFI_SCAN_DONE_BIT;
    if (!done && millis() - scanStartedAt < SCAN_TIMEOUT_MS) return;
    int n = done ? WiFi.scanComplete() : WIFI_SCAN_FAILED;
    // One entry per network name (strongest access point), strongest first, at most 20
    struct Net {
      String ssid;
      int rssi;
      bool locked;
    };
    std::vector<Net> nets;
    for (int i = 0; i < n; i++) {
      String ssid = WiFi.SSID(i);
      if (!ssid.length()) continue;  // hidden network
      bool seen = false;
      for (Net &net : nets) {
        if (net.ssid != ssid) continue;
        net.rssi = max(net.rssi, (int)WiFi.RSSI(i));
        seen = true;
      }
      if (!seen) nets.push_back({ssid, WiFi.RSSI(i), WiFi.encryptionType(i) != WIFI_AUTH_OPEN});
    }
    std::sort(nets.begin(), nets.end(), [](const Net &a, const Net &b) { return a.rssi > b.rssi; });
    String json = "[";
    for (size_t i = 0; i < nets.size() && i < 20; i++) {
      json += String(i ? "," : "") + "{\"s\":\"" + jsonEscape(nets[i].ssid) + "\",\"r\":" + nets[i].rssi +
              ",\"l\":" + (nets[i].locked ? "true" : "false") + "}";
    }
    scanResults = json + "]";
    WiFi.scanDelete();
    scanState = n < 0 ? SCAN_FAILED : SCAN_DONE;
  } else {
    return;
  }
  if (scanRadioOnly) {
    scanRadioOnly = false;
    if (wifiState == WIFI_STANDBY) WiFi.mode(WIFI_OFF);
  }
}

// Address and interface the board is reachable on right now
bool onWifi() { return !linkUp && wifiState != WIFI_STANDBY; }

IPAddress currentIp() {
  if (!onWifi()) return ETH.localIP();
  if (wifiState == WIFI_HOTSPOT) return WiFi.softAPIP();
  return WiFi.localIP();
}

String netModeText() {
  if (!linkUp && wifiState == WIFI_STANDBY) return "no cable";
  if (!onWifi()) return usingFallback ? "Ethernet, fallback IP" : "Ethernet";
  if (wifiState == WIFI_HOTSPOT) return "hotspot";
  if (wifiState == WIFI_JOINED) return "WiFi";
  return "joining WiFi";
}

String wifiText() {
  const String &ssid = settings.wifiSsid;
  switch (wifiState) {
    case WIFI_JOINING: return "joining " + ssid + "...";
    case WIFI_JOINED: return ssid + " (" + String(WiFi.RSSI()) + " dBm)";
    case WIFI_HOTSPOT:
      return "hotspot " + runningName + ", " + String(WiFi.softAPgetStationNum()) + " connected";
    default: return ssid.length() ? "standby, backup: " + ssid : "standby, backup: hotspot";
  }
}

void startNetworkServicesOnce() {
  if (networkServicesStarted || !(gotIp || wifiGotIp || wifiState == WIFI_HOTSPOT)) return;
  networkServicesStarted = true;
  if (MDNS.begin(settings.name.c_str())) MDNS.addService("http", "tcp", 80);
  ArduinoOTA.setHostname(settings.name.c_str());
  ArduinoOTA.setMdnsEnabled(false);  // mDNS already started above
  ArduinoOTA.onStart([]() { Serial.println("OTA update starting"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA update done, restarting"); });
  ArduinoOTA.begin();
  Serial.printf("Web interface: http://%s.local  or  http://%s\n",
                settings.name.c_str(), currentIp().toString().c_str());
}

// ---------------------------------------------------------------- Art-Net

void sendPollReply(IPAddress to) {
  uint8_t r[239] = {0};
  IPAddress ip = currentIp();
  uint8_t mac[6];
  ETH.macAddress(mac);  // WiFi uses the same MAC
  bool anyData = false;
  for (const Relay &relay : relays) anyData |= relay.value >= 0;

  memcpy(r, "Art-Net", 8);
  r[8] = OP_POLL_REPLY & 0xFF;
  r[9] = OP_POLL_REPLY >> 8;
  for (int i = 0; i < 4; i++) r[10 + i] = ip[i];
  r[14] = ARTNET_PORT & 0xFF;
  r[15] = ARTNET_PORT >> 8;
  r[16] = 0;                                // firmware version hi
  r[17] = 2;                                // firmware version lo
  r[18] = (settings.universe >> 8) & 0x7F;  // Net
  r[19] = (settings.universe >> 4) & 0x0F;  // SubNet
  r[20] = 0x00;                             // OEM unknown
  r[21] = 0xFF;
  r[23] = 0xE0;                             // Status1: indicators normal, ports set from network
  strncpy((char *)&r[26], settings.name.c_str(), 17);
  snprintf((char *)&r[44], 64, "ArtSwitch %d relays - ch %u-%u", RELAY_COUNT, settings.baseChannel,
           settings.baseChannel + RELAY_COUNT - 1);
  char states[RELAY_COUNT * 4 + 1] = "";
  for (int i = 0; i < RELAY_COUNT; i++)
    strcat(states, relays[i].pin == NO_PIN ? " --" : relays[i].on ? " ON" : " OFF");
  snprintf((char *)&r[108], 64, "#0001 [%04lu] relays%s", (unsigned long)(pollReplies % 10000), states);
  r[173] = 1;                               // one port
  r[174] = 0x80;                            // port can output DMX512
  r[182] = anyData ? 0x80 : 0x00;           // GoodOutput: data received
  r[190] = settings.universe & 0x0F;        // SwOut: Universe
  r[200] = 0x00;                            // Style: StNode
  memcpy(&r[201], mac, 6);
  for (int i = 0; i < 4; i++) r[207 + i] = ip[i];
  r[211] = 1;                               // BindIndex
  // Status2: web config, DHCP capable, 15-bit port-address, currently DHCP?
  bool dhcp = onWifi() ? wifiState != WIFI_HOTSPOT : !usingFallback;
  r[212] = 0x01 | 0x04 | 0x08 | (dhcp ? 0x02 : 0x00);

  udp.beginPacket(to, ARTNET_PORT);
  udp.write(r, sizeof(r));
  udp.endPacket();
  pollReplies++;
}

void handleArtnet() {
  int len = udp.parsePacket();
  if (len <= 0) return;
  if (len > (int)sizeof(packet)) len = sizeof(packet);
  len = udp.read(packet, len);
  // Drop whatever didn't fit: WiFiUDP ignores new packets until the current one is fully consumed
  udp.flush();
  if (len < 12 || memcmp(packet, "Art-Net", 8) != 0) return;

  uint16_t op = packet[8] | (packet[9] << 8);

  bool artnetOn = settings.protocol != PROTO_SACN;
  if (op == OP_POLL) {
    // On sACN only, stay out of Art-Net discovery so no controller sends Art-Net that would be ignored
    if (!artnetOn) return;
    lastPollFrom = udp.remoteIP();
    lastPollAt = millis();
    sendPollReply(udp.remoteIP());
  } else if (op == OP_DMX && len >= 18) {
    uint16_t portAddress = packet[14] | ((packet[15] & 0x7F) << 8);
    noteTraffic(udp.remoteIP(), portAddress, false);
    if (!artnetOn || portAddress != settings.universe) return;
    uint16_t dmxLength = (packet[16] << 8) | packet[17];
    applyDmxFrame(&packet[18], min((int)dmxLength, len - 18));
  }
}

// ---------------------------------------------------------------- sACN (E1.31)

IPAddress sacnMulticastIp(uint16_t universe) { return IPAddress(239, 255, universe >> 8, universe & 0xFF); }

// Listen on the sACN port and join the universe's multicast group. A group is joined on the
// interfaces that are up at that moment, so this runs again whenever Ethernet or WiFi comes up.
void updateSacnSocket() {
  char key[24] = "";
  if (settings.protocol != PROTO_ARTNET && (gotIp || wifiGotIp || wifiState == WIFI_HOTSPOT))
    snprintf(key, sizeof(key), "%u/%d%d%d%d", settings.sacnUniverse, linkUp, gotIp, (int)wifiState, wifiGotIp);
  if (sacnJoinedKey == key) return;
  sacnJoinedKey = key;
  if (sacnSocket >= 0) close(sacnSocket);
  sacnSocket = -1;
  sacnSourceCount = 0;
  if (!key[0]) return;
  sacnSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(SACN_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (sacnSocket < 0 || bind(sacnSocket, (sockaddr *)&addr, sizeof(addr)) != 0) {
    Serial.println("sACN: can't open port 5568");
    if (sacnSocket >= 0) close(sacnSocket);
    sacnSocket = -1;
    return;
  }
  fcntl(sacnSocket, F_SETFL, O_NONBLOCK);
  // Our universe and the discovery universe; interface "any" joins on every interface that is up
  for (uint16_t universe : {settings.sacnUniverse, SACN_DISCOVERY_UNIVERSE}) {
    ip_mreq group = {};
    group.imr_multiaddr.s_addr = (uint32_t)sacnMulticastIp(universe);
    group.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(sacnSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &group, sizeof(group));
  }
  Serial.printf("sACN universe %u (%s)\n", settings.sacnUniverse,
                sacnMulticastIp(settings.sacnUniverse).toString().c_str());
}

// Universe Discovery packet: framing vector 2, discovery vector 1, then pages of universe numbers
void noteSacnAnnounce(IPAddress ip, int len) {
  if (len < 120 || (packet[40] << 24 | packet[41] << 16 | packet[42] << 8 | packet[43]) != 2 ||
      (packet[114] << 24 | packet[115] << 16 | packet[116] << 8 | packet[117]) != 1)
    return;
  int slot = -1;
  for (int i = 0; i < announceCount; i++)
    if (memcmp(announces[i].cid, &packet[22], 16) == 0) slot = i;
  if (slot < 0) {
    if (announceCount < 4) {
      slot = announceCount++;
    } else {  // replace the stalest source
      slot = 0;
      for (int i = 1; i < 4; i++)
        if (announces[i].lastAt < announces[slot].lastAt) slot = i;
    }
    memcpy(announces[slot].cid, &packet[22], 16);
  }
  SacnAnnounce &a = announces[slot];
  a.ip = ip;
  copySacnName(a.name, &packet[44]);
  if (packet[118] == 0) a.count = 0;  // the first page starts the list again
  for (int i = 120; i + 1 < len; i += 2) {
    if (a.count < 16) a.universes[a.count] = packet[i] << 8 | packet[i + 1];
    a.count++;
  }
  a.lastAt = millis();
}

// Track the source and decide whether to follow this packet: the highest priority source that
// is still sending wins, packets arriving out of order are dropped (E1.31 section 6.7.2)
bool acceptSacnSource(const uint8_t *cid, uint8_t priority, uint8_t sequence, bool terminated) {
  uint32_t now = millis();
  for (int i = 0; i < sacnSourceCount;) {  // forget sources that stopped sending
    if (now - sacnSources[i].lastAt > SACN_SOURCE_TIMEOUT_MS) sacnSources[i] = sacnSources[--sacnSourceCount];
    else i++;
  }
  int slot = -1;
  for (int i = 0; i < sacnSourceCount; i++)
    if (memcmp(sacnSources[i].cid, cid, 16) == 0) slot = i;
  if (terminated) {
    if (slot >= 0) sacnSources[slot] = sacnSources[--sacnSourceCount];
    return false;
  }
  if (slot >= 0) {
    int8_t diff = sequence - sacnSources[slot].sequence;
    if (diff <= 0 && diff > -20) return false;
  } else {
    if (sacnSourceCount == 4) return false;  // more sources than anyone needs for three relays
    slot = sacnSourceCount++;
    memcpy(sacnSources[slot].cid, cid, 16);
  }
  sacnSources[slot].priority = priority;
  sacnSources[slot].sequence = sequence;
  sacnSources[slot].lastAt = now;
  for (int i = 0; i < sacnSourceCount; i++)
    if (sacnSources[i].priority > priority) return false;
  return true;
}

void handleSacn() {
  if (sacnSocket < 0) return;
  sockaddr_in from;
  socklen_t fromLen = sizeof(from);
  int len = recvfrom(sacnSocket, packet, sizeof(packet), 0, (sockaddr *)&from, &fromLen);
  if (len <= 0) return;
  IPAddress fromIp(from.sin_addr.s_addr);
  static const uint8_t ACN_ID[] = {0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00};
  auto be32 = [](const uint8_t *p) { return (uint32_t)p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3]; };
  if (len < 22 || memcmp(&packet[4], ACN_ID, sizeof(ACN_ID)) != 0) return;
  // Root vector 8 = extended packet: universe discovery (sync packets are ignored)
  if (be32(&packet[18]) == 8) {
    noteSacnAnnounce(fromIp, len);
    return;
  }
  // Root vector 4 = data packet, framing vector 2, DMP vector 2 with address/data type 0xa1
  if (len < 126 || be32(&packet[18]) != 4 || be32(&packet[40]) != 2 || packet[117] != 2 || packet[118] != 0xa1)
    return;
  uint16_t universe = packet[113] << 8 | packet[114];
  noteTraffic(fromIp, universe, true, &packet[44], packet[108]);
  uint8_t options = packet[112];
  if (universe != settings.sacnUniverse || (options & 0x80)) return;  // other universe, or preview data
  if (!acceptSacnSource(&packet[22], packet[108], packet[111], options & 0x40)) return;
  if (packet[125] != 0) return;  // not DMX (start code)
  uint16_t slots = (packet[123] << 8 | packet[124]) - 1;
  applyDmxFrame(&packet[126], min((int)slots, len - 126));
}

// ---------------------------------------------------------------- OSC

uint32_t oscMessages = 0;
uint32_t lastOscFrom = 0;
uint32_t lastOscAt = 0;

// Listen on the OSC port while OSC is on; the socket follows the board across Ethernet and WiFi
void updateOscSocket() {
  uint16_t port = settings.oscEnabled ? settings.oscPort : 0;
  if (port == oscOpenPort) return;
  oscUdp.stop();
  if (port) oscUdp.begin(port);
  oscOpenPort = port;
}

// OSC strings are null-terminated and padded to a multiple of 4 bytes; returns the offset after it
int oscSkipString(const uint8_t *p, int at, int len) {
  while (at < len && p[at]) at++;
  return (at + 4) & ~3;
}

// Same switch points as DMX: a 0-1 fader goes through on/off levels with the dead zone between
void applyOscLevel(int i, int level) {
  const RelayConfig &c = settings.relay[i];
  if (!relays[i].want && level >= c.onAt) requestRelay(i, true);
  else if (relays[i].want && level <= c.offAt) requestRelay(i, false);
}

// One message: /<name>/relay/<n> or /artswitch/relay/<n> with an int, float, double or true/false
void handleOscMessage(const uint8_t *p, int len) {
  int typesAt = oscSkipString(p, 0, len);
  if (typesAt + 1 >= len || p[typesAt] != ',') return;
  const char *address = (const char *)p;
  int n = -1;
  for (const String &prefix : {"/" + runningName + "/relay/", String("/") + DEFAULT_NAME + "/relay/"}) {
    if (strncmp(address, prefix.c_str(), prefix.length()) == 0 && strlen(address) == prefix.length() + 1)
      n = address[prefix.length()] - '1';
  }
  if (n < 0 || n >= RELAY_COUNT) return;
  char type = p[typesAt + 1];
  int argAt = oscSkipString(p, typesAt, len);
  auto be32 = [&](int at) { return (uint32_t)p[at] << 24 | p[at + 1] << 16 | p[at + 2] << 8 | p[at + 3]; };
  int level;
  if (type == 'T') level = 255;
  else if (type == 'F') level = 0;
  else if (type == 'i' && argAt + 4 <= len) level = (int32_t)be32(argAt) ? 255 : 0;
  else if (type == 'f' && argAt + 4 <= len) {
    uint32_t bits = be32(argAt);
    float f;
    memcpy(&f, &bits, 4);
    level = lroundf(constrain(f, 0.0f, 1.0f) * 255);
  } else if (type == 'd' && argAt + 8 <= len) {
    uint64_t bits = (uint64_t)be32(argAt) << 32 | be32(argAt + 4);
    double d;
    memcpy(&d, &bits, 8);
    level = lround(constrain(d, 0.0, 1.0) * 255);
  } else {
    return;
  }
  oscMessages++;
  lastOscAt = millis();
  applyOscLevel(n, level);
}

// A bundle holds size-prefixed messages or further bundles; depth keeps a hostile packet from recursing far
void handleOscPacket(const uint8_t *p, int len, int depth = 0) {
  if (len >= 16 && memcmp(p, "#bundle", 8) == 0) {
    if (depth >= 4) return;
    for (int at = 16; at + 4 <= len;) {  // skip "#bundle" and the 8-byte time tag
      int size = p[at] << 24 | p[at + 1] << 16 | p[at + 2] << 8 | p[at + 3];
      at += 4;
      if (size <= 0 || at + size > len) return;
      handleOscPacket(p + at, size, depth + 1);
      at += size;
    }
  } else if (len >= 8 && p[0] == '/') {
    handleOscMessage(p, len);
  }
}

void handleOsc() {
  if (!oscOpenPort) return;
  int len = oscUdp.parsePacket();
  if (len <= 0) return;
  if (len > (int)sizeof(packet)) len = sizeof(packet);
  len = oscUdp.read(packet, len);
  oscUdp.flush();
  lastOscFrom = oscUdp.remoteIP();
  handleOscPacket(packet, len);
}

// Every change of a relay output, whatever caused it, goes to the feedback address
void sendOscFeedback(int i) {
  if (!oscOpenPort || !(uint32_t)settings.oscFeedbackIp) return;
  uint8_t msg[64] = {0};
  int len = snprintf((char *)msg, 48, "/%s/relay/%d", runningName.c_str(), i + 1);
  len = (len + 4) & ~3;
  memcpy(&msg[len], ",i", 2);
  len += 4;
  msg[len + 3] = relays[i].on ? 1 : 0;
  len += 4;
  oscUdp.beginPacket(settings.oscFeedbackIp, settings.oscFeedbackPort);
  oscUdp.write(msg, len);
  oscUdp.endPacket();
}

// ---------------------------------------------------------------- web

const char PAGE[] PROGMEM = R"html(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>%RUNNINGNAME% - ArtSwitch</title>
<style>
body{font-family:system-ui,sans-serif;max-width:480px;margin:0 auto;padding:16px;background:#f4f4f5;color:#18181b}
.card{background:#fff;border-radius:10px;padding:16px;margin-bottom:16px;box-shadow:0 1px 3px #0002}
h1{font-size:1.4em;margin:0 0 12px}h2{font-size:1.05em;margin:0 0 10px}
table{width:100%;border-collapse:collapse}td{padding:4px 0}td:last-child{text-align:right;font-family:monospace}
label{display:block;margin:10px 0 4px;font-size:.9em}input,select{width:100%;padding:8px;box-sizing:border-box;font-size:1em}
small{color:#71717a}button{padding:10px 16px;font-size:1em;border:0;border-radius:6px;background:#2563eb;color:#fff;margin:8px 8px 0 0}
button.off{background:#52525b}
.relay{display:flex;align-items:center;justify-content:space-between;padding:8px 0;border-bottom:1px solid #e4e4e7}
.sw{position:relative;display:inline-block;flex:none;width:52px;height:30px;margin:0 0 0 12px;cursor:pointer}
.sw input{position:absolute;inset:0;width:100%;height:100%;margin:0;padding:0;opacity:0;cursor:pointer;z-index:1}
.sw span{position:absolute;inset:0;border-radius:15px;background:#d4d4d8;transition:background .2s}
.sw span::after{content:'';position:absolute;left:3px;top:3px;width:24px;height:24px;border-radius:50%;background:#fff;box-shadow:0 1px 3px #0005;transition:transform .2s}
.sw input:checked+span{background:#16a34a}.sw input:checked+span::after{transform:translateX(22px)}
.sw input:focus-visible+span{outline:2px solid #2563eb;outline-offset:2px}
.relay.wait .sw span{background:#fcd34d}
#offline{display:none;background:#fee2e2;color:#991b1b;border-radius:6px;padding:10px;margin-bottom:16px;font-size:.9em}
body.offline #offline{display:block}body.offline #relays{opacity:.5}
.relay:last-of-type{border-bottom:0}fieldset{border:1px solid #e4e4e7;border-radius:8px;margin:12px 0 0;padding:4px 12px 12px}
legend{font-weight:bold;font-size:.9em;padding:0 4px}
.dual{position:relative;height:32px;margin-top:4px}
.dual .trk{position:absolute;left:0;right:0;top:13px;height:6px;border-radius:3px}
.dual input{position:absolute;left:0;top:6px;width:100%;margin:0;padding:0;height:20px;background:none;pointer-events:none;-webkit-appearance:none;appearance:none}
.dual input::-webkit-slider-runnable-track{background:none}.dual input::-moz-range-track{background:none}
.dual input::-webkit-slider-thumb{-webkit-appearance:none;pointer-events:auto;width:22px;height:22px;border-radius:50%;background:#fff;border:2px solid #2563eb;cursor:pointer}
.dual input::-moz-range-thumb{pointer-events:auto;width:18px;height:18px;border-radius:50%;background:#fff;border:2px solid #2563eb;cursor:pointer}
.dl{display:flex;justify-content:space-between;font-size:.85em}.dl .dz{color:#b45309}
dialog{border:0;border-radius:12px;padding:20px;width:calc(100% - 32px);max-width:380px;box-shadow:0 10px 40px #0004}
dialog::backdrop{background:#0006}dialog h2{margin-top:0}.err{color:#b91c1c;font-size:.9em;min-height:1.2em}
.url{font-family:monospace;color:#2563eb;word-break:break-all}
.row{display:flex;gap:8px}.row button{margin:0;flex:none}
label.chk{display:flex;align-items:center;gap:8px;margin-top:14px}label.chk input{width:auto;margin:0}
.nets{border:1px solid #e4e4e7;border-radius:8px;margin-top:6px;max-height:260px;overflow:auto}.nets:empty{display:none}
.nets div{display:flex;justify-content:space-between;gap:8px;padding:9px 10px;border-bottom:1px solid #f4f4f5}
.nets div.pick{cursor:pointer}.nets div.pick:hover{background:#eff6ff}.nets div:last-child{border-bottom:0}
.top{background:#1e3a8a;color:#fff;border-radius:10px;padding:14px 16px;margin-bottom:16px}
.top .lbl{font-size:.75em;letter-spacing:.08em;text-transform:uppercase;opacity:.7}
.top .nm{font-size:1.8em;font-weight:bold;line-height:1.2;word-break:break-all}
.top .ad{font-family:monospace;font-size:1.15em;margin-top:4px}.top .ad small{color:#bfdbfe}
.top .lo{font-family:monospace;font-size:.9em;color:#bfdbfe}.warn button{margin:8px 0 0}.warn{background:#fef3c7;color:#92400e;border-radius:6px;padding:10px;font-size:.9em}.relay .st{font-family:monospace;font-size:.85em;text-align:right;margin-left:auto}
</style></head><body>
<header class="top"><div class="lbl">ArtSwitch</div><div class="nm">%RUNNINGNAME%</div>
<div class="ad"><span id="hip">%IP%</span> <small id="hmode">%IPMODE%</small></div>
<div class="ad"><span id="huni">%UNIVERSETEXT%</span> <small>&middot;</small> <span id="hdmx">DMX %DMXRANGE%</span></div>
<div class="lo">http://%RUNNINGNAME%.local</div></header>
<dialog id="nameDlg"><form method="dialog" onsubmit="return false">
<h2>Name this board</h2>
<p>Every ArtSwitch starts out as <b>artswitch</b>. Give this one its own name, so it can't be
mixed up with other boards on the network.</p>
<input id="newName" maxlength="31" placeholder="e.g. stage-left" autocomplete="off">
<p><small>Web address:</small><br><span class="url" id="newUrl">&nbsp;</span></p>
<div class="err" id="nameErr"></div>
<button type="button" onclick="saveName()">Save &amp; restart</button><button type="button" class="off" onclick="nameDlg.close()">Later</button>
</form></dialog>
<div id="offline">No answer from the board, retrying&hellip; The values below may be out of date.</div>
<div class="card"><h2>Relays</h2><div id="relays"></div>
<small>Incoming DMX data overrides the switches. Amber: the switch waits for the minimum switch time.</small></div>
<div class="card"><h2>Status</h2><table>
<tr><td>Last DMX data</td><td id="age">-</td></tr>
<tr><td>Packets received</td><td id="packets">-</td></tr>
<tr><td>ArtPoll (discovery)</td><td id="polls">-</td></tr>
<tr><td>OSC</td><td id="osc">-</td></tr>
<tr><td>WiFi</td><td id="wifi">-</td></tr>
<tr><td>MAC</td><td id="mac">-</td></tr>
<tr><td colspan="2"><b>DMX sources seen (any universe)</b></td></tr>
<tbody id="traffic"><tr><td>none yet</td><td></td></tr></tbody>
<tbody id="announce"></tbody>
</table></div>
<div class="card"><h2>Settings</h2><form method="post" action="/save" id="settingsForm">
%NAMEWARNING%%RENAMEPENDING%
<label>Device name</label><input name="name" value="%NAME%" maxlength="31" required>
<small>Must be changed from "artswitch" before saving or restarting. Use a name that is unique on
the network: the web address becomes http://name.local. Letters, digits and "-". Applied after restart.</small>
<label>Protocol</label><select name="proto">%PROTOOPTIONS%</select>
<label>Art-Net universe (port-address 0-32767)</label><input name="universe" type="number" min="0" max="32767" value="%UNIVERSE%">
<small>0 = Net 0, Sub-Net 0, Universe 0. Some software (e.g. QLC+) calls this "universe 1".</small>
<label>sACN universe (1-63999)</label><input name="sacnuni" type="number" min="1" max="63999" value="%SACNUNIVERSE%">
<small>Used when the protocol includes sACN. The board receives multicast (239.255.x.x) and unicast sACN.
With "Art-Net and sACN" the most recent packet sets the relays.</small>
<label>DMX base address (1-510)</label><input name="channel" type="number" min="1" max="510" value="%CHANNEL%">
<small>Relay 1 = base, relay 2 = base+1, relay 3 = base+2.</small>
%RELAYSETTINGS%
<small>Relay settings apply immediately; a freed pin is switched off and released.
Between "off" and "on" a relay keeps its state. Minimum switch time (0-60 s, 0 = no limit) delays
a switch until that long after the previous one, protecting relays and loads from fast chases.</small>
<label>Fallback IP when no DHCP</label><input name="fallback" value="%FALLBACK%" placeholder="automatic: %AUTOFALLBACK%">
<small>Leave empty for automatic (%AUTOFALLBACK%, netmask 255.255.0.0, unique per board).
Used when no DHCP server answers within 10 s. A DHCP server appearing in the next 2 minutes
still takes over; after that the address is held until the cable is re-plugged. Applied after restart.</small>
<label>Backup WiFi network</label><div class="row"><input name="wifissid" value="%WIFISSID%" maxlength="32" placeholder="none: hotspot only" autocomplete="off">
<button type="button" id="scanBtn" onclick="scan()">Scan</button></div><div class="nets" id="nets"></div>
<label>WiFi password</label><input name="wifipass" type="password" maxlength="63" placeholder="%WIFIPASSHINT%" autocomplete="new-password">
<small>Used only while the Ethernet cable is unplugged: the board joins this network (with its Ethernet
MAC, so the router gives it the same IP) and goes back to Ethernet when the cable returns. If the network can't be joined within 30 s,
or none is set, the board opens an open WiFi hotspot named "%RUNNINGNAME%"; connect to it and open
http://192.168.4.1. Leave the password empty to keep the saved one. Applied immediately.</small>
<label class="chk"><input type="checkbox" name="osc" value="1"%OSCCHECKED%> Accept OSC</label>
<label>OSC port</label><input name="oscport" type="number" min="1" max="65535" value="%OSCPORT%">
<label>OSC feedback to (IP:port)</label><input name="oscfb" value="%OSCFB%" placeholder="none, e.g. 192.168.1.20:9000">
<small>Send <b>/%RUNNINGNAME%/relay/1</b> (to 3) to switch this board, or <b>/artswitch/relay/1</b> to switch every board
that listens on the port. Value 1 or 0, true or false, or a fader from 0 to 1 (uses the switch points above).
The latest command wins, so incoming DMX that crosses a switch point overrides OSC. With a feedback address,
every change of a relay is sent there as /%RUNNINGNAME%/relay/n 1 or 0. Applied immediately.</small>
<br><button type="submit">Save</button><button type="button" class="off" onclick="restart()">Restart</button>
</form></div>
<script>
// Status is polled one request at a time, so an old answer can never overwrite a newer one.
// gen changes around every switch click; a poll started before that is stale and not shown.
let gen=0,polling=false,again=false,timer=0,rowsKey='';
async function toggle(n,on){gen++;
try{await fetch('/relay?n='+n+'&state='+(on?'on':'off'),{cache:'no-store'})}catch(e){}
gen++;poll()}
// Two-handle threshold slider: grey = off zone, amber = dead zone (relay keeps its state), green = on zone.
// The handles can't cross; dragging one into the other pushes it along so off stays below on.
document.querySelectorAll('.dual').forEach(d=>{const lo=d.querySelector('.lo'),hi=d.querySelector('.hi'),
trk=d.querySelector('.trk'),lab=d.nextElementSibling,pct=v=>v+' ('+Math.round(v/2.55)+'%)';
function upd(e){let a=+lo.value,b=+hi.value;
if(a>=b){if(e&&e.target===lo){b=Math.min(255,a+1);a=b-1}else{a=Math.max(0,b-1);b=a+1}lo.value=a;hi.value=b}
if(e){lo.style.zIndex=e.target===lo?3:2;hi.style.zIndex=e.target===hi?3:2}
const p=a/255*100,q=b/255*100;
trk.style.background='linear-gradient(to right,#a1a1aa '+p+'%,#f59e0b '+p+'%,#f59e0b '+q+'%,#16a34a '+q+'%)';
lab.querySelector('.vo').textContent=pct(a);lab.querySelector('.vn').textContent=pct(b)}
lo.addEventListener('input',upd);hi.addEventListener('input',upd);upd()});
// WiFi scan: start it, then ask until the list is ready (it waits for a join attempt in progress)
// Source names come from the network: show them as text, never as HTML
const esc=t=>t.replace(/[&<>"]/g,c=>'&#'+c.charCodeAt(0)+';');
const getJson=async u=>(await fetch(u,{cache:'no-store'})).json();
async function scan(){scanBtn.disabled=true;scanBtn.textContent='Scanning…';nets.innerHTML='';
try{let s=await getJson('/scan?start=1');
for(let i=0;s.state==='scanning'&&i<60;i++){await new Promise(t=>setTimeout(t,700));s=await getJson('/scan')}
if(s.state!=='done')throw 0;
if(!s.nets.length)nets.innerHTML='<div>No networks found</div>';
s.nets.forEach(n=>{const d=document.createElement('div');d.className='pick';
d.innerHTML='<span></span><small>'+n.r+' dBm &middot; '+(n.l?'password':'open')+'</small>';d.firstChild.textContent=n.s;
d.onclick=()=>{settingsForm.wifissid.value=n.s;settingsForm.wifipass.value='';nets.innerHTML='';if(n.l)settingsForm.wifipass.focus()};
nets.appendChild(d)})}
catch(e){nets.innerHTML='<div>Scan failed, try again.</div>'}
scanBtn.disabled=false;scanBtn.textContent='Scan'}
let savedName='%NAME%';const runningName='%RUNNINGNAME%',defaultName='%DEFAULTNAME%';
// Same cleanup as the board does: lowercase, letters/digits/'-', no '-' at the ends
const cleanName=s=>s.toLowerCase().replace(/[^a-z0-9-]/g,'').replace(/^-+|-+$/g,'').slice(0,31);
newName.addEventListener('input',()=>{const c=cleanName(newName.value);
newUrl.textContent=c?'http://'+c+'.local':' ';nameErr.textContent=c===defaultName?'Choose something other than "'+defaultName+'".':''});
function askName(){newName.value='';newUrl.innerHTML='&nbsp;';nameErr.textContent='';nameDlg.showModal();newName.focus()}
newName.addEventListener('keydown',e=>{if(e.key==='Enter')saveName()});
async function saveName(){const c=cleanName(newName.value);
if(!c||c===defaultName){nameErr.textContent='Enter a name other than "'+defaultName+'".';return}
const r=await fetch('/name',{method:'POST',body:new URLSearchParams({name:c})});
if(!r.ok){nameErr.textContent=await r.text();return}
savedName=c;nameDlg.close();await new Promise(t=>setTimeout(t,500));restart(false)}
settingsForm.addEventListener('submit',e=>{if(savedName===defaultName){e.preventDefault();askName()}});
if(savedName===defaultName)askName();
// After a restart, follow the board to its (possibly new) .local name, or the same IP
async function restart(ask=true){if(savedName===defaultName){askName();return}
if(ask&&!confirm('Restart?'))return;
const r=await fetch('/restart');if(!r.ok){alert(await r.text());return}
const target=location.hostname.endsWith('.local')?'http://'+savedName+'.local/':location.origin+'/';
document.body.innerHTML='<h1>ArtSwitch</h1><div class="card">Restarting, waiting for <b>'+target+'</b> &hellip;</div>';
await new Promise(t=>setTimeout(t,3000));
for(let i=0;i<60;i++){try{await fetch(target+'status',{mode:'no-cors',cache:'no-store'});location.href=target;return}catch(e){}
await new Promise(t=>setTimeout(t,1000))}
document.body.innerHTML+='<div class="card">Board did not come back within a minute. Try <a href="'+target+'">'+target+'</a> or its IP address.</div>'}
if(new URLSearchParams(location.search).has('saved')){history.replaceState(null,'','/');
if(savedName!==runningName&&confirm('Name changed to '+savedName+'. Restart now to move to http://'+savedName+'.local?'))restart(false)}
// Rows are only rebuilt when a pin is assigned or freed; otherwise they are updated in place,
// so a click never lands on an element that is being replaced
function showRelays(s){const key=s.relays.map(r=>r.pin<0).join();
if(key!==rowsKey){rowsKey=key;relays.innerHTML=s.relays.map((r,i)=>'<div class="relay"><div><b>Relay '+(i+1)+
'</b><br><small class="info"></small></div><span class="st"></span>'+(r.pin<0?'':'<label class="sw"><input type="checkbox" role="switch" aria-label="Relay '+
(i+1)+'" onchange="toggle('+(i+1)+',this.checked)"><span></span></label>')+'</div>').join('')}
s.relays.forEach((r,i)=>{const row=relays.children[i],sw=row.querySelector('input');
row.querySelector('.info').innerHTML=(r.pin<0?'no pin':r.pinName)+', ch '+r.ch+' &middot; on &ge;'+r.onAt+' / off &le;'+r.offAt+
(r.min?' &middot; min '+(r.min/1000)+' s':'');
row.querySelector('.st').innerHTML=r.pin<0?'not used':(r.value<0?'':'DMX '+r.value)+
(r.wait?'<br>'+(r.want?'on':'off')+' in '+(r.wait/1000).toFixed(1)+' s':'');
row.classList.toggle('wait',r.wait>0);if(sw)sw.checked=r.want})}
async function poll(){if(polling){again=true;return}polling=true;again=false;clearTimeout(timer);const g=gen;
try{const res=await fetch('/status',{cache:'no-store',signal:AbortSignal.timeout(3000)});if(!res.ok)throw 0;
const s=await res.json();document.body.classList.remove('offline');if(g===gen)show(s)}
catch(e){document.body.classList.add('offline')}
polling=false;timer=setTimeout(poll,again?0:1000)}
function show(s){showRelays(s);
age.textContent=s.age<0?'never':(s.age/1000).toFixed(1)+' s ago';
packets.textContent=s.packets;hip.textContent=s.ip;
polls.textContent=s.polls?s.polls+'×, last from '+s.pollFrom+' '+(s.pollAge/1000).toFixed(0)+' s ago':'none yet';hmode.textContent=s.net;
huni.textContent=s.uniText;hdmx.textContent='DMX '+s.base+'–'+(s.base+s.relays.length-1);mac.textContent=s.mac;
wifi.textContent=s.wifi;
osc.textContent=s.oscPort?'port '+s.oscPort+', '+(s.oscMsgs?s.oscMsgs+'×, last from '+s.oscFrom+' '+(s.oscAge/1000).toFixed(0)+' s ago':'nothing yet'):'off';
if(s.traffic.length)traffic.innerHTML=s.traffic.map(t=>'<tr><td>'+t.ip+' &rarr; '+(t.s?'sACN':'Art-Net')+' universe '+t.u+(t.s?'<br><small>'+esc(t.n||'unnamed')+', priority '+t.p+'</small>':'')+'</td><td>'+(t.age/1000).toFixed(1)+' s ago</td></tr>').join('');
announce.innerHTML=s.announce.length?'<tr><td colspan="2"><b>sACN universes announced</b></td></tr>'+s.announce.map(a=>'<tr><td>'+esc(a.n||'unnamed')+' ('+a.ip+')<br><small>'+(a.u.length+a.more>1?'universes ':'universe ')+a.u.join(', ')+(a.more?' and '+a.more+' more':'')+'</small></td><td>'+(a.age/1000).toFixed(0)+' s ago</td></tr>').join(''):''}
poll();
</script></body></html>)html";

String renderRelaySettings() {
  String html;
  for (int i = 0; i < RELAY_COUNT; i++) {
    const RelayConfig &c = settings.relay[i];
    String n = String(i + 1);
    html += "<fieldset><legend>Relay " + n + " (channel " + String(settings.baseChannel + i) + ")</legend>";
    html += "<label>Pin</label><select name=\"pin" + n + "\">";
    html += "<option value=\"-1\"" + String(c.pin == NO_PIN ? " selected" : "") + ">Not used</option>";
    for (const PinOption &o : PIN_OPTIONS) {
      html += "<option value=\"" + String(o.pin) + "\"" + (c.pin == o.pin ? " selected" : "") +
              ">" + o.name + o.note + "</option>";
    }
    html += "</select><label>Switch points</label><div class=\"dual\"><div class=\"trk\"></div>";
    html += "<input type=\"range\" class=\"lo\" name=\"off" + n + "\" min=\"0\" max=\"255\" value=\"" +
            String(c.offAt) + "\" aria-label=\"Relay " + n + " off at\">";
    html += "<input type=\"range\" class=\"hi\" name=\"on" + n + "\" min=\"0\" max=\"255\" value=\"" +
            String(c.onAt) + "\" aria-label=\"Relay " + n + " on at\"></div>";
    html += "<div class=\"dl\"><span>Off &le; <b class=\"vo\"></b></span><span class=\"dz\">dead zone</span>"
            "<span>On &ge; <b class=\"vn\"></b></span></div>";
    html += "<label>Minimum switch time (s)</label><input name=\"min" + n +
            "\" type=\"number\" min=\"0\" max=\"60\" step=\"0.1\" value=\"" + String(c.minSwitchMs / 1000.0f, 1) +
            "\"></fieldset>";
  }
  return html;
}

// Header line: the universe(s) the board listens to
String universeText() {
  String artnet = "Art-Net " + String(settings.universe), sacn = "sACN " + String(settings.sacnUniverse);
  if (settings.protocol == PROTO_ARTNET) return artnet;
  if (settings.protocol == PROTO_SACN) return sacn;
  return artnet + " / " + sacn;
}

String renderPage() {
  String page = FPSTR(PAGE);
  page.replace("%UNIVERSETEXT%", universeText());
  page.replace("%UNIVERSE%", String(settings.universe));
  page.replace("%SACNUNIVERSE%", String(settings.sacnUniverse));
  String protoOptions;
  for (int i = PROTO_ARTNET; i <= PROTO_BOTH; i++)
    protoOptions += "<option value=\"" + String(i) + "\"" + (settings.protocol == i ? " selected" : "") + ">" +
                    PROTOCOL_NAMES[i] + "</option>";
  page.replace("%PROTOOPTIONS%", protoOptions);
  page.replace("%CHANNEL%", String(settings.baseChannel));
  page.replace("%RELAYSETTINGS%", renderRelaySettings());
  page.replace("%NAMEWARNING%", settings.name == DEFAULT_NAME
                                    ? "<div class=\"warn\">This board still has the default name. Give it its "
                                      "own name before you can save settings or restart.<br>"
                                      "<button type=\"button\" onclick=\"askName()\">Name this board</button></div>"
                                    : "");
  page.replace("%RENAMEPENDING%", settings.name != runningName
                                      ? "<div class=\"warn\">New name saved. Restart to move to http://" +
                                            settings.name + ".local</div>"
                                      : String());
  page.replace("%RUNNINGNAME%", runningName);
  page.replace("%DEFAULTNAME%", DEFAULT_NAME);
  page.replace("%IP%", currentIp().toString());
  page.replace("%DMXRANGE%", String(settings.baseChannel) + "&ndash;" + String(settings.baseChannel + RELAY_COUNT - 1));
  page.replace("%IPMODE%", netModeText());
  page.replace("%NAME%", settings.name);
  page.replace("%OSCCHECKED%", settings.oscEnabled ? " checked" : "");
  page.replace("%OSCPORT%", String(settings.oscPort));
  page.replace("%OSCFB%", (uint32_t)settings.oscFeedbackIp
                              ? settings.oscFeedbackIp.toString() + ":" + String(settings.oscFeedbackPort)
                              : String());
  page.replace("%WIFIPASSHINT%", settings.wifiPass.length() ? "saved, leave empty to keep" : "none (open network)");
  page.replace("%WIFISSID%", htmlEscape(settings.wifiSsid));  // last: an SSID may contain anything
  page.replace("%FALLBACK%", (uint32_t)settings.fallbackIp ? settings.fallbackIp.toString() : String());
  page.replace("%AUTOFALLBACK%", autoFallbackIp().toString());
  return page;
}

String renderStatus() {
  static char json[3072];  // static: too big for the web server task's stack; requests run one at a time
  size_t n = snprintf(json, sizeof(json), "{\"relays\":[");
  // Keeps n inside the buffer if a long WiFi name ever fills it, so the output is cut, not overrun
  auto clamp = [&]() { n = min(n, sizeof(json) - 1); };
  for (int i = 0; i < RELAY_COUNT; i++) {
    const RelayConfig &c = settings.relay[i];
    clamp();
    n += snprintf(json + n, sizeof(json) - n,
                  "%s{\"pin\":%d,\"pinName\":\"%s\",\"ch\":%u,\"on\":%s,\"want\":%s,\"wait\":%lu,"
                  "\"value\":%d,\"onAt\":%u,\"offAt\":%u,\"min\":%u}",
                  i ? "," : "", relays[i].pin, pinName(relays[i].pin), settings.baseChannel + i,
                  relays[i].on ? "true" : "false", relays[i].want ? "true" : "false",
                  (unsigned long)lockoutRemainingMs(i), relays[i].value, c.onAt, c.offAt, c.minSwitchMs);
  }
  clamp();
  n += snprintf(json + n, sizeof(json) - n,
                "],\"universe\":%u,\"uniText\":\"%s\",\"base\":%u,\"polls\":%lu,\"pollFrom\":\"%s\",\"pollAge\":%lu,"
                "\"age\":%ld,\"packets\":%lu,\"ip\":\"%s\",\"net\":\"%s\",\"mac\":\"%s\",\"wifi\":\"%s\","
                "\"oscPort\":%u,\"oscMsgs\":%lu,\"oscFrom\":\"%s\",\"oscAge\":%lu,"
                "\"traffic\":[",
                settings.universe, universeText().c_str(), settings.baseChannel, (unsigned long)pollReplies,
                IPAddress(lastPollFrom).toString().c_str(), (unsigned long)(millis() - lastPollAt),
                dmxPackets == 0 ? -1L : (long)(millis() - lastDmxAt), (unsigned long)dmxPackets,
                currentIp().toString().c_str(), netModeText().c_str(), ETH.macAddress().c_str(),
                jsonEscape(wifiText()).c_str(), oscOpenPort, (unsigned long)oscMessages,
                IPAddress(lastOscFrom).toString().c_str(), (unsigned long)(millis() - lastOscAt));
  uint32_t now = millis();
  for (int i = 0; i < trafficCount; i++) {
    clamp();
    n += snprintf(json + n, sizeof(json) - n, "%s{\"ip\":\"%s\",\"u\":%u,\"s\":%d,\"n\":\"%s\",\"p\":%u,\"age\":%lu}", i ? "," : "",
                  IPAddress(traffic[i].ip).toString().c_str(), traffic[i].universe, traffic[i].sacn,
                  jsonEscape(traffic[i].name).c_str(), traffic[i].priority,
                  (unsigned long)(now - traffic[i].lastAt));
  }
  clamp();
  n += snprintf(json + n, sizeof(json) - n, "],\"announce\":[");
  bool first = true;
  for (int i = 0; i < announceCount; i++) {
    const SacnAnnounce &a = announces[i];
    if (now - a.lastAt > SACN_ANNOUNCE_TIMEOUT_MS) continue;
    String list;
    for (int u = 0; u < min((int)a.count, 16); u++) list += String(u ? "," : "") + a.universes[u];
    clamp();
    n += snprintf(json + n, sizeof(json) - n, "%s{\"ip\":\"%s\",\"n\":\"%s\",\"u\":[%s],\"more\":%d,\"age\":%lu}",
                  first ? "" : ",", IPAddress(a.ip).toString().c_str(), jsonEscape(a.name).c_str(), list.c_str(),
                  max(0, a.count - 16), (unsigned long)(now - a.lastAt));
    first = false;
  }
  clamp();
  snprintf(json + n, sizeof(json) - n, "]}");
  return String(json);
}

void handleSave(AsyncWebServerRequest *request) {
  auto arg = [request](const String &name) {
    const AsyncWebParameter *p = request->getParam(name, true);
    return p ? p->value() : String();
  };
  long universe = arg("universe").toInt();
  long sacnUniverse = arg("sacnuni").length() ? arg("sacnuni").toInt() : 1;
  long protocol = arg("proto").toInt();
  long channel = arg("channel").toInt();
  IPAddress fallback;
  String name = sanitizeName(arg("name"));
  String wifiSsid = arg("wifissid");
  wifiSsid.trim();
  String wifiPass = arg("wifipass");
  // An empty password field keeps the saved password, unless the network changes
  if (!wifiPass.length() && wifiSsid == settings.wifiSsid) wifiPass = settings.wifiPass;
  if (!wifiSsid.length()) wifiPass = "";
  bool oscEnabled = arg("osc") == "1";
  long oscPort = arg("oscport").length() ? arg("oscport").toInt() : 8000;
  String oscFb = arg("oscfb");
  oscFb.trim();
  IPAddress oscFbIp;
  long oscFbPort = 9000;
  bool oscFbValid = true;
  if (oscFb.length()) {
    int colon = oscFb.indexOf(':');
    oscFbValid = colon > 0 && oscFbIp.fromString(oscFb.substring(0, colon));
    oscFbPort = colon > 0 ? oscFb.substring(colon + 1).toInt() : 0;
  }
  String error;
  if (settingsPending) error = "Previous save still being applied, try again.";
  else if (name == DEFAULT_NAME) error = "Give this board its own name first, not \"artswitch\".";
  else if (universe < 0 || universe > 32767) error = "Universe must be 0-32767.";
  else if (sacnUniverse < 1 || sacnUniverse > SACN_MAX_UNIVERSE) error = "sACN universe must be 1-63999.";
  else if (protocol < PROTO_ARTNET || protocol > PROTO_BOTH) error = "Unknown protocol.";
  else if (channel < 1 || channel > 512 - RELAY_COUNT + 1) error = "Base address must be 1-510.";
  else if (arg("fallback").length() && !fallback.fromString(arg("fallback")))
    error = "Fallback IP is not a valid address.";
  else if (wifiSsid.length() > 32) error = "WiFi network name is longer than 32 characters.";
  else if (wifiPass.length() && (wifiPass.length() < 8 || wifiPass.length() > 63))
    error = "WiFi password must be 8-63 characters, or empty for an open network.";
  else if (oscPort < 1 || oscPort > 65535) error = "OSC port must be 1-65535.";
  else if (!oscFbValid || oscFbPort < 1 || oscFbPort > 65535)
    error = "OSC feedback must be an IP address and port, like 192.168.1.20:9000, or empty.";

  RelayConfig relayConfig[RELAY_COUNT];
  for (int i = 0; i < RELAY_COUNT && !error.length(); i++) {
    String n = String(i + 1);
    String relay = "Relay " + n + ": ";
    String pin = arg("pin" + n);
    long onAt = arg("on" + n).length() ? arg("on" + n).toInt() : DEFAULT_ON_AT;
    long offAt = arg("off" + n).length() ? arg("off" + n).toInt() : DEFAULT_OFF_AT;
    long minMs = lround(arg("min" + n).toFloat() * 1000);
    RelayConfig &c = relayConfig[i];
    c.pin = pin.length() ? pin.toInt() : NO_PIN;
    if (!isAllowedPin(c.pin)) error = relay + "pin not allowed.";
    else if (onAt < 1 || onAt > 255 || offAt < 0 || offAt > 254) error = relay + "thresholds must be 0-255.";
    else if (offAt >= onAt) error = relay + "\"off at\" must be lower than \"on at\".";
    else if (minMs < 0 || minMs > MAX_MIN_SWITCH_MS) error = relay + "minimum switch time must be 0-60 s.";
    for (int j = 0; j < i && !error.length(); j++)
      if (c.pin != NO_PIN && c.pin == relayConfig[j].pin)
        error = "Relay " + String(j + 1) + " and relay " + n + " use the same pin.";
    c.onAt = onAt;
    c.offAt = offAt;
    c.minSwitchMs = minMs;
  }
  if (error.length()) {
    request->send(400, "text/plain", error + " Go back and check the settings.");
    return;
  }

  pendingSettings.protocol = protocol;
  pendingSettings.universe = universe;
  pendingSettings.sacnUniverse = sacnUniverse;
  pendingSettings.baseChannel = channel;
  memcpy(pendingSettings.relay, relayConfig, sizeof(relayConfig));
  pendingSettings.name = name;
  pendingSettings.fallbackIp = fallback;
  pendingSettings.wifiSsid = wifiSsid;
  pendingSettings.wifiPass = wifiPass;
  pendingSettings.oscEnabled = oscEnabled;
  pendingSettings.oscPort = oscPort;
  pendingSettings.oscFeedbackIp = oscFbIp;
  pendingSettings.oscFeedbackPort = oscFbPort;
  settingsPending = true;
  request->redirect("/?saved=1");
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) { r->send(200, "text/html", renderPage()); });
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *r) {
    AsyncWebServerResponse *res = r->beginResponse(200, "application/json", renderStatus());
    res->addHeader("Cache-Control", "no-store");
    r->send(res);
  });
  server.on("/save", HTTP_POST, handleSave);
  // Name only, from the naming pop-up; everything else stays as it is
  server.on("/name", HTTP_POST, [](AsyncWebServerRequest *r) {
    const AsyncWebParameter *p = r->getParam("name", true);
    String name = sanitizeName(p ? p->value() : String());
    if (settingsPending) {
      r->send(400, "text/plain", "Previous save still being applied, try again.");
      return;
    }
    if (name == DEFAULT_NAME) {
      r->send(400, "text/plain", "Choose a name other than \"artswitch\".");
      return;
    }
    pendingSettings = settings;
    pendingSettings.name = name;
    settingsPending = true;
    r->send(200, "text/plain", name);
  });
  server.on("/relay", HTTP_GET, [](AsyncWebServerRequest *r) {
    const AsyncWebParameter *n = r->getParam("n");
    const AsyncWebParameter *state = r->getParam("state");
    int i = n ? n->value().toInt() - 1 : -1;
    if (i < 0 || i >= RELAY_COUNT || !state) {
      r->send(400, "text/plain", "Use /relay?n=1..3&state=on|off");
      return;
    }
    pendingTest[i] = state->value() == "on" ? 1 : 0;
    r->send(200, "text/plain", "OK");
  });
  // /scan?start=1 starts a WiFi scan; /scan reports "scanning" until the list is ready
  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (r->hasParam("start") && scanState != SCAN_REQUESTED && scanState != SCAN_RUNNING)
      scanState = SCAN_REQUESTED;
    String json = scanState == SCAN_DONE     ? "{\"state\":\"done\",\"nets\":" + scanResults + "}"
                  : scanState == SCAN_FAILED ? String("{\"state\":\"failed\"}")
                                             : String("{\"state\":\"scanning\"}");
    AsyncWebServerResponse *res = r->beginResponse(200, "application/json", json);
    res->addHeader("Cache-Control", "no-store");
    r->send(res);
  });
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (settings.name == DEFAULT_NAME) {
      r->send(400, "text/plain", "Give this board its own name and save before restarting.");
      return;
    }
    r->send(200, "text/plain", "Restarting");
    restartRequested = true;
  });
  server.onNotFound([](AsyncWebServerRequest *r) {
    // On the hotspot, phones probe a known page to check for internet; sending them to the
    // settings page makes them offer "Sign in to network", which opens it
    if (wifiState == WIFI_HOTSPOT) {
      r->redirect("http://" + WiFi.softAPIP().toString() + "/");
      return;
    }
    r->send(404, "text/plain", "Not found");
  });
  server.begin();
}

void applyPendingSettings() {
  if (!settingsPending) return;
  bool wifiChanged = settings.wifiSsid != pendingSettings.wifiSsid || settings.wifiPass != pendingSettings.wifiPass;
  settings = pendingSettings;
  saveSettings();
  if (wifiChanged) stopWifi();  // updateWifi() starts it again with the new network if still needed
  for (int i = 0; i < RELAY_COUNT; i++) {
    assignRelayPin(i, settings.relay[i].pin);
    relays[i].value = -1;
  }
  settingsPending = false;
  Serial.printf("Saved: %s, base channel %u, name %s, fallback %s\n", universeText().c_str(),
                settings.baseChannel, settings.name.c_str(), settings.fallbackIp.toString().c_str());
  for (int i = 0; i < RELAY_COUNT; i++) {
    const RelayConfig &c = settings.relay[i];
    Serial.printf("  relay %d: pin %d, on >= %u, off <= %u, min %u ms\n", i + 1, c.pin, c.onAt, c.offAt,
                  c.minSwitchMs);
  }
}

// ---------------------------------------------------------------- main

void setup() {
  Serial.begin(115200);
  Serial.println("\nArtSwitch starting");
  loadSettings();
  runningName = settings.name;
  digitalWrite(RELAY1_LED_PIN, HIGH);  // LED off
  pinMode(RELAY1_LED_PIN, OUTPUT);
  for (int i = 0; i < RELAY_COUNT; i++) assignRelayPin(i, settings.relay[i].pin);
  Serial.printf("Universe %u, base channel %u, name %s\n", settings.universe, settings.baseChannel,
                settings.name.c_str());

  WiFi.onEvent(onNetworkEvent);
  WiFi.persistent(false);  // the network is kept in our own settings; don't rewrite WiFi's flash copy
  ETH.begin();  // pins and PHY come from the wt32-eth01 board variant

  udp.begin(ARTNET_PORT);

  setupWebServer();
}

void loop() {
  checkDhcpFallback();
  if (dhcpRestartRequested) restartDhcp();
  probeForDhcpServer();
  updateWifi();
  updateScan();
  startNetworkServicesOnce();
  handleArtnet();
  updateSacnSocket();
  handleSacn();
  updateOscSocket();
  handleOsc();
  applyPendingSettings();
  applyPendingTests();
  for (int i = 0; i < RELAY_COUNT; i++) updateRelay(i);  // carry out switches held back by min time
  digitalWrite(RELAY1_LED_PIN, relays[0].on ? LOW : HIGH);
  if (networkServicesStarted) ArduinoOTA.handle();
  if (restartRequested) {
    delay(200);  // let the response go out
    ESP.restart();
  }
}
