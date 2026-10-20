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

#include <Arduino.h>
#include <ETH.h>
#include <WiFiUdp.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <esp_mac.h>

// Every board starts with this name; saving or restarting is refused until it is changed,
// so two boards on one network don't both answer to artswitch.local
static const char *DEFAULT_NAME = "artswitch";
static const int RELAY_COUNT = 3;
static const int8_t NO_PIN = -1;
// Onboard green LED4 sits on IO17 (TXD2) and lights when the pin is low (datasheet V1.4 4.2,
// verified on the board). It mirrors relay 1's output for testing.
static const int RELAY1_LED_PIN = 17;
static const uint16_t ARTNET_PORT = 6454;
static const uint32_t DHCP_TIMEOUT_MS = 10000;
static const uint32_t DHCP_PROBE_INTERVAL_MS = 15000;
// Once someone may have configured against the fallback address, it must not move mid-show
static const uint32_t DHCP_GRACE_MS = 120000;

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
  uint16_t universe;          // 15-bit Art-Net port-address (Net:SubNet:Universe)
  uint16_t baseChannel;       // relay n listens on baseChannel + n
  RelayConfig relay[RELAY_COUNT];
  IPAddress fallbackIp;       // used when DHCP fails; 0.0.0.0 = automatic 169.254.x.x
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
AsyncWebServer server(80);

// Web handlers run in the async TCP task; they hand changes to loop() through these
Settings pendingSettings;
volatile bool settingsPending = false;
volatile bool restartRequested = false;
volatile int8_t pendingTest[RELAY_COUNT] = {-1, -1, -1};  // -1 none, 0 off, 1 on

volatile bool linkUp = false;
volatile bool gotIp = false;
uint32_t linkUpAt = 0;
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

uint32_t lastDmxAt = 0;
uint32_t dmxPackets = 0;
uint32_t pollReplies = 0;
uint32_t lastPollFrom = 0;  // shown on the web page to check controller discovery
uint32_t lastPollAt = 0;

// Recently seen Art-Net DMX sources, any universe - shown on the web page for troubleshooting
struct TrafficSeen {
  uint32_t ip;
  uint16_t universe;
  uint32_t lastAt;
};
TrafficSeen traffic[8];
int trafficCount = 0;

void noteTraffic(IPAddress ip, uint16_t universe) {
  int slot = -1;
  for (int i = 0; i < trafficCount; i++)
    if (traffic[i].ip == (uint32_t)ip && traffic[i].universe == universe) slot = i;
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
  }
  traffic[slot].lastAt = millis();
}

uint8_t packet[600];

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
  settings.universe = prefs.getUShort("universe", 0);
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
  prefs.end();
}

void saveSettings() {
  prefs.begin("artswitch", false);
  prefs.putString("name", settings.name);
  prefs.putUShort("universe", settings.universe);
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
  prefs.end();
}

// ---------------------------------------------------------------- relays

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

void startNetworkServicesOnce() {
  if (networkServicesStarted || !gotIp) return;
  networkServicesStarted = true;
  if (MDNS.begin(settings.name.c_str())) MDNS.addService("http", "tcp", 80);
  ArduinoOTA.setHostname(settings.name.c_str());
  ArduinoOTA.setMdnsEnabled(false);  // mDNS already started above
  ArduinoOTA.onStart([]() { Serial.println("OTA update starting"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA update done, restarting"); });
  ArduinoOTA.begin();
  Serial.printf("Web interface: http://%s.local  or  http://%s\n",
                settings.name.c_str(), ETH.localIP().toString().c_str());
}

// ---------------------------------------------------------------- Art-Net

void sendPollReply(IPAddress to) {
  uint8_t r[239] = {0};
  IPAddress ip = ETH.localIP();
  uint8_t mac[6];
  ETH.macAddress(mac);
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
  r[212] = 0x01 | 0x04 | 0x08 | (usingFallback ? 0x00 : 0x02);

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

  if (op == OP_POLL) {
    lastPollFrom = udp.remoteIP();
    lastPollAt = millis();
    sendPollReply(udp.remoteIP());
  } else if (op == OP_DMX && len >= 18) {
    uint16_t portAddress = packet[14] | ((packet[15] & 0x7F) << 8);
    noteTraffic(udp.remoteIP(), portAddress);
    if (portAddress != settings.universe) return;
    uint16_t dmxLength = (packet[16] << 8) | packet[17];
    bool used = false;
    for (int i = 0; i < RELAY_COUNT; i++) {
      uint16_t channel = settings.baseChannel + i;
      int index = 18 + channel - 1;
      if (channel > dmxLength || index >= len) continue;
      applyDmxValue(i, packet[index]);
      used = true;
    }
    if (used) {
      dmxPackets++;
      lastDmxAt = millis();
    }
  }
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
.top{background:#1e3a8a;color:#fff;border-radius:10px;padding:14px 16px;margin-bottom:16px}
.top .lbl{font-size:.75em;letter-spacing:.08em;text-transform:uppercase;opacity:.7}
.top .nm{font-size:1.8em;font-weight:bold;line-height:1.2;word-break:break-all}
.top .ad{font-family:monospace;font-size:1.15em;margin-top:4px}.top .ad small{color:#bfdbfe}
.top .lo{font-family:monospace;font-size:.9em;color:#bfdbfe}.warn button{margin:8px 0 0}.warn{background:#fef3c7;color:#92400e;border-radius:6px;padding:10px;font-size:.9em}.relay .st{font-family:monospace;font-size:.85em;text-align:right;margin-left:auto}
</style></head><body>
<header class="top"><div class="lbl">ArtSwitch</div><div class="nm">%RUNNINGNAME%</div>
<div class="ad"><span id="hip">%IP%</span> <small id="hmode">%IPMODE%</small></div>
<div class="ad"><span id="huni">Universe %UNIVERSE%</span> <small>&middot;</small> <span id="hdmx">DMX %DMXRANGE%</span></div>
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
<small>Incoming Art-Net overrides the switches. Amber: the switch waits for the minimum switch time.</small></div>
<div class="card"><h2>Status</h2><table>
<tr><td>Last Art-Net</td><td id="age">-</td></tr>
<tr><td>Packets received</td><td id="packets">-</td></tr>
<tr><td>ArtPoll (discovery)</td><td id="polls">-</td></tr>
<tr><td>MAC</td><td id="mac">-</td></tr>
<tr><td colspan="2"><b>Art-Net seen (any universe)</b></td></tr>
<tbody id="traffic"><tr><td>none yet</td><td></td></tr></tbody>
</table></div>
<div class="card"><h2>Settings</h2><form method="post" action="/save" id="settingsForm">
%NAMEWARNING%%RENAMEPENDING%
<label>Device name</label><input name="name" value="%NAME%" maxlength="31" required>
<small>Must be changed from "artswitch" before saving or restarting. Use a name that is unique on
the network: the web address becomes http://name.local. Letters, digits and "-". Applied after restart.</small>
<label>Art-Net universe (port-address 0-32767)</label><input name="universe" type="number" min="0" max="32767" value="%UNIVERSE%">
<small>0 = Net 0, Sub-Net 0, Universe 0. Some software (e.g. QLC+) calls this "universe 1".</small>
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
polls.textContent=s.polls?s.polls+'×, last from '+s.pollFrom+' '+(s.pollAge/1000).toFixed(0)+' s ago':'none yet';hmode.textContent=s.fallback?'fallback':'DHCP';
huni.textContent='Universe '+s.universe;hdmx.textContent='DMX '+s.base+'–'+(s.base+s.relays.length-1);mac.textContent=s.mac;
if(s.traffic.length)traffic.innerHTML=s.traffic.map(t=>'<tr><td>'+t.ip+' &rarr; universe '+t.u+'</td><td>'+(t.age/1000).toFixed(1)+' s ago</td></tr>').join('')}
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

String renderPage() {
  String page = FPSTR(PAGE);
  page.replace("%UNIVERSE%", String(settings.universe));
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
  page.replace("%IP%", ETH.localIP().toString());
  page.replace("%DMXRANGE%", String(settings.baseChannel) + "&ndash;" + String(settings.baseChannel + RELAY_COUNT - 1));
  page.replace("%IPMODE%", usingFallback ? "fallback" : "DHCP");
  page.replace("%NAME%", settings.name);
  page.replace("%FALLBACK%", (uint32_t)settings.fallbackIp ? settings.fallbackIp.toString() : String());
  page.replace("%AUTOFALLBACK%", autoFallbackIp().toString());
  return page;
}

String renderStatus() {
  char json[1600];
  size_t n = snprintf(json, sizeof(json), "{\"relays\":[");
  for (int i = 0; i < RELAY_COUNT; i++) {
    const RelayConfig &c = settings.relay[i];
    n += snprintf(json + n, sizeof(json) - n,
                  "%s{\"pin\":%d,\"pinName\":\"%s\",\"ch\":%u,\"on\":%s,\"want\":%s,\"wait\":%lu,"
                  "\"value\":%d,\"onAt\":%u,\"offAt\":%u,\"min\":%u}",
                  i ? "," : "", relays[i].pin, pinName(relays[i].pin), settings.baseChannel + i,
                  relays[i].on ? "true" : "false", relays[i].want ? "true" : "false",
                  (unsigned long)lockoutRemainingMs(i), relays[i].value, c.onAt, c.offAt, c.minSwitchMs);
  }
  n += snprintf(json + n, sizeof(json) - n,
                "],\"universe\":%u,\"base\":%u,\"polls\":%lu,\"pollFrom\":\"%s\",\"pollAge\":%lu,"
                "\"age\":%ld,\"packets\":%lu,\"ip\":\"%s\",\"fallback\":%s,\"mac\":\"%s\",\"traffic\":[",
                settings.universe, settings.baseChannel, (unsigned long)pollReplies,
                IPAddress(lastPollFrom).toString().c_str(), (unsigned long)(millis() - lastPollAt),
                dmxPackets == 0 ? -1L : (long)(millis() - lastDmxAt), (unsigned long)dmxPackets,
                ETH.localIP().toString().c_str(), usingFallback ? "true" : "false", ETH.macAddress().c_str());
  uint32_t now = millis();
  for (int i = 0; i < trafficCount; i++) {
    n += snprintf(json + n, sizeof(json) - n, "%s{\"ip\":\"%s\",\"u\":%u,\"age\":%lu}", i ? "," : "",
                  IPAddress(traffic[i].ip).toString().c_str(), traffic[i].universe,
                  (unsigned long)(now - traffic[i].lastAt));
  }
  snprintf(json + n, sizeof(json) - n, "]}");
  return String(json);
}

void handleSave(AsyncWebServerRequest *request) {
  auto arg = [request](const String &name) {
    const AsyncWebParameter *p = request->getParam(name, true);
    return p ? p->value() : String();
  };
  long universe = arg("universe").toInt();
  long channel = arg("channel").toInt();
  IPAddress fallback;
  String name = sanitizeName(arg("name"));
  String error;
  if (settingsPending) error = "Previous save still being applied, try again.";
  else if (name == DEFAULT_NAME) error = "Give this board its own name first, not \"artswitch\".";
  else if (universe < 0 || universe > 32767) error = "Universe must be 0-32767.";
  else if (channel < 1 || channel > 512 - RELAY_COUNT + 1) error = "Base address must be 1-510.";
  else if (arg("fallback").length() && !fallback.fromString(arg("fallback")))
    error = "Fallback IP is not a valid address.";

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

  pendingSettings.universe = universe;
  pendingSettings.baseChannel = channel;
  memcpy(pendingSettings.relay, relayConfig, sizeof(relayConfig));
  pendingSettings.name = name;
  pendingSettings.fallbackIp = fallback;
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
  server.on("/restart", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (settings.name == DEFAULT_NAME) {
      r->send(400, "text/plain", "Give this board its own name and save before restarting.");
      return;
    }
    r->send(200, "text/plain", "Restarting");
    restartRequested = true;
  });
  server.onNotFound([](AsyncWebServerRequest *r) { r->send(404, "text/plain", "Not found"); });
  server.begin();
}

void applyPendingSettings() {
  if (!settingsPending) return;
  settings = pendingSettings;
  saveSettings();
  for (int i = 0; i < RELAY_COUNT; i++) {
    assignRelayPin(i, settings.relay[i].pin);
    relays[i].value = -1;
  }
  settingsPending = false;
  Serial.printf("Saved: universe %u, base channel %u, name %s, fallback %s\n", settings.universe,
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
  ETH.begin();  // pins and PHY come from the wt32-eth01 board variant

  udp.begin(ARTNET_PORT);

  setupWebServer();
}

void loop() {
  checkDhcpFallback();
  if (dhcpRestartRequested) restartDhcp();
  probeForDhcpServer();
  startNetworkServicesOnce();
  handleArtnet();
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
