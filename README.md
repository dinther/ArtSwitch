# ArtSwitch

Art-Net controlled relay switch on a **WT32-ETH01** (ESP32 + Ethernet) board. Up to three relays, each switched by one DMX channel, configured from a web page in your browser.

- Up to 3 relays on consecutive DMX channels from a base address
- Relay pins chosen on the web page — no reflashing
- DHCP, with a self-assigned `169.254.x.x` fallback when there is no DHCP server
- Reachable by name: `http://<name>.local`
- Shows up in Art-Net tools (MadMapper, QLC+, DMX-Workshop) via ArtPoll
- Firmware updates over Ethernet

Tested on WT32-ETH01 **V1.4** boards with MadMapper as the Art-Net source.

### What you need

- **WT32-ETH01** board (Wireless-Tag; ESP32 + LAN8720 Ethernet)
- **5 V power supply** for the board
- **Relay module(s)** with their own driver input, or relays with a transistor and flyback diode each
- **3.3 V USB-to-serial adapter** for the first flash only (the board has no USB port)
- [PlatformIO](https://platformio.org/) to build and upload the firmware

---

## How it switches

Each relay has two switch points, set per relay with a two-handle slider on the web page:

| DMX value | Relay | Default |
|---|---|---|
| at or above **on** | **on** | 140 – 255 |
| at or below **off** | **off** | 0 – 115 |
| in between (the *dead zone*) | stays as it was | 116 – 139 |

The dead zone stops a relay from chattering when a fader sits near a switch point. Useful settings:

| Off ≤ / On ≥ | Behaviour |
|---|---|
| 115 / 140 | Default: switches around halfway |
| 0 / 1 | On as soon as the fader leaves zero |
| 254 / 255 | On only at full |

**Minimum switch time** (per relay, 0–60 s, default 0 = no limit): after a relay switches, it won't switch again until this time has passed. A change asked for in the meantime is carried out when the time is up — if the DMX value still asks for it. A chase or strobe flashing the channel can't then hammer the relay or the load (motors, compressors, transformers, some LED drivers). The relay card shows a countdown ("OFF in 1.4 s") while a switch is held back. The first switch after power-up is never delayed. The test buttons obey the minimum time too.

- **Relay 1** listens on the base address, **relay 2** on base + 1, **relay 3** on base + 2.
- At power-up all relays are **off**.
- When Art-Net stops (controller off, cable pulled), relays **hold their last state**.
- Outputs are **active-high**: the pin goes to 3.3 V when the relay is on.

---

## Wiring

### Power

Power the board from **one** source only: 5 V into the **5V** pin (and **GND**).

If the relay coils need 12 V or 24 V, use a separate 5 V regulator for the board. Connect all grounds together (board, relay supply, relay module).

### Relays

A GPIO pin gives 3.3 V at a few milliamps — **not** enough to drive a relay coil directly. Use either:

- a **relay module** with its own driver (pins usually labelled VCC / GND / IN), or
- a **transistor + flyback diode** per relay.

Connect the relay's input to one of the pins below, and its ground to the board's GND.

### Which pins to use

The V1.4 board labels some pins by function instead of GPIO number. The web page shows both.

| Pin on web page | Printed on V1.4 board | Notes |
|---|---|---|
| IO4 | IO4 | Recommended (default for relay 1) |
| IO32 (CFG) | CFG | Recommended |
| IO33 (485_EN) | 485_EN | Recommended |
| IO2 | IO2 | Must not be pulled high while flashing over USB |
| IO5 (RXD) | RXD | May pulse briefly at boot — relay may click |
| IO14 | IO14 | May pulse briefly at boot — relay may click |
| IO15 | IO15 | May pulse briefly at boot — relay may click |

Not offered: **IO17 (TXD)** (drives the onboard relay 1 LED, see below), **IO12** (a pull-up on it stops the board from booting), IO35 / IO36 / IO39 (input only), TXO / RXO / IO0 (programming), and the pins used by Ethernet.

### Onboard LEDs

Besides the power LED and the green/yellow LEDs in the network jack, the board has two small green LEDs side by side next to the RXD/TXD pins (LED3 on IO5, LED4 on IO17).

- **LED4 (IO17)** shows **relay 1**: lit while relay 1 is on. Handy for testing without a relay connected.
- **LED3 (IO5)** is not used by the firmware.

---

## First-time setup

### 1. Flash the firmware

A new board needs the firmware flashed once over USB serial — see [Over USB serial](#over-usb-serial-first-flash-or-recovery). After that, updates go over Ethernet.

### 2. Connect and find the board

Plug in Ethernet, then power. Open in a browser:

**http://artswitch.local**

If that doesn't load, find the board's IP address — see [Finding a board](#finding-a-board).

### 3. Give it a name

A new board is called `artswitch`. The first time you open its page, a **Name this board** pop-up appears. Type a name that's unique on your network, for example `stage-left`; the pop-up shows the web address it will get. Click **Save & restart**: the board restarts and the page reopens at `http://<new-name>.local`.

Clicked **Later**? Everything else works, but **settings can't be saved and the board can't be restarted until it has its own name** — so two boards never fight over the same address. Save, Restart and the **Name this board** button at the top of Settings bring the pop-up back.

Names use letters, digits and `-` (not at the start or end); capitals become lowercase. To rename a board later, change the name in Settings, Save, and accept the restart.

### 4. Set the Art-Net address

| Setting | Meaning |
|---|---|
| **Art-Net universe** | 15-bit port-address, 0–32767. Universe 0 is what some software (e.g. QLC+) calls "universe 1". In MadMapper the numbers match: universe 10 in MadMapper = 10 here. |
| **DMX base address** | Channel for relay 1. Relays 2 and 3 use the next two channels. |
| **Relay 1/2/3 pin** | Which pin each relay is wired to, or **Not used**. |
| **Switch points** | Two-handle slider: left handle = off, right handle = on. Grey = off zone, amber = dead zone, green = on zone. Values shown as DMX value and percent. The handles can't cross. |
| **Minimum switch time** | Seconds between switches, 0 = no limit. See [How it switches](#how-it-switches). |

Universe, base address, pins, switch points and minimum switch time take effect **immediately** on Save. When you move a relay to another pin, the old pin is switched off and released first.

### 5. Test

The **Relays** card has **ON / OFF** buttons per relay. Incoming Art-Net overrides them straight away — to test by hand, stop your controller's output first.

---

## The web page

**Header** — the board's name, IP address (DHCP or fallback), Art-Net universe and DMX channels, and its `http://<name>.local` address. Updates live, e.g. right after you save a new universe. The browser tab is titled with the board's name too.

**Relays** — state of each relay, its pin, channel, switch points and minimum switch time, the last DMX value received, a countdown when a switch is being held back, and test buttons.

**Status**
- *Last Art-Net* / *Packets received* — data for this board's universe and channels.
- *ArtPoll (discovery)* — how many discovery requests arrived from controllers such as MadMapper, and from where.
- *MAC* — the board's Ethernet MAC address.
- *Art-Net seen (any universe)* — every source and universe that reaches the board, matching or not. **First place to look when a relay doesn't respond**: it tells you whether data arrives at all, and on which universe.

**Settings** — name, universe, base address, per relay its pin, switch points and minimum switch time, and the fallback IP. **Restart** applies name and fallback IP changes; after a restart the page follows the board to its (new) address.

---

## Network

### DHCP and the 169.254 fallback

Normally your router (DHCP server) gives the board an address.

When there's no DHCP server — for example a cable straight from the board to a laptop — the board waits 10 seconds, then gives itself a fixed **169.254.x.x** address. Windows and macOS do the same when they get no DHCP answer, so a laptop on a direct cable ends up in the same network automatically (this can take Windows about a minute).

The 169.254 address is derived from the board's MAC address: **the same every boot, and different per board**. The web page shows it in the *Fallback IP* field ("automatic: 169.254.x.x") — write it on a label on the board.

> **Note:** for **2 minutes** after taking its 169.254 address, the board keeps listening for a late DHCP server (for example a router that boots slower than the board after a power cut). If one shows up, the board switches to the address it offers. After those 2 minutes the address is **fixed** until the cable is re-plugged or the board restarts, so it can't change mid-show.

You can also type your own fallback address in the *Fallback IP* field (leave it empty for automatic). The netmask follows from the address: `2.x` / `10.x` → 255.0.0.0, `169.254.x` → 255.255.0.0, `192.168.x` → 255.255.255.0. Takes effect after restart.

### Finding a board

- **By name:** `http://<name>.local`.
- **Art-Net tools:** QLC+, DMX-Workshop and MadMapper (with *Use Unicast* and *Auto Poll for devices* on) list each board with its current IP.
- **Your router:** the DHCP client list shows the board's name and IP.
- **The label:** the 169.254 fallback address never changes for a board.

Older Android phones can't open `.local` names; use the IP address there.

### Several boards

Give every board its own name (enforced — see first-time setup) and, usually, its own base address or universe. Boards with the same universe and base address switch together.

---

## Using it with MadMapper

1. **Preferences → Project → DMX**: set *DMX Output Device* to **ArtNet** and *Interface* to the network adapter the boards are on.
2. Add a DMX fixture on the universe and channel(s) set on the board.
3. Make sure the fixture is **switched on** in MadMapper — a disabled fixture sends nothing.
4. Check the board's web page: *Art-Net seen* should show your PC's IP and the universe.

**Broadcast or unicast.** With *Use Unicast* off (the default), MadMapper broadcasts every universe to the whole network and each board picks out its own — no device list needed. With **Use Unicast** on and **Auto Poll for devices** ticked, MadMapper discovers the boards and lists them with IP, status, name and universe; it then sends each universe only to the boards that use it, which keeps a busy network quieter. The board's *ArtPoll (discovery)* line on the Status card shows whether MadMapper's discovery requests arrive.

---

## Building and flashing

The project builds with [PlatformIO](https://platformio.org/) (Arduino framework, Arduino-ESP32 2.0.x). Libraries are fetched automatically. Commands below use `pio`; if that isn't on your PATH, use `python -m platformio` instead.

| Environment | Use |
|---|---|
| `wt32-eth01` | Upload over USB serial |
| `ota` | Upload over Ethernet |

Settings (name, universe, pins, …) are stored in flash and survive firmware updates.

### Over Ethernet (normal)

```
pio run -e ota -t upload --upload-port <name>.local
```

or use the board's IP address instead of the name.

### Over USB serial (first flash, or recovery)

Needed only for a brand-new board, or if a network update ever leaves the board unreachable. You need a **3.3 V USB-to-serial adapter**.

| Adapter | WT32-ETH01 |
|---|---|
| TX | RXO |
| RX | TXO |
| GND | GND |

Leave the adapter's 3V3/5V wire **disconnected** and power the board from its own 5 V supply. Powering the board from the adapter makes the adapter drop out ("A device attached to the system is not functioning").

1. Connect **IO0 to GND**.
2. Power-cycle the board. It now waits for new firmware.
3. Run (replace COM5 with your adapter's port, e.g. `/dev/ttyUSB0` on Linux):
   ```
   pio run -e wt32-eth01 -t upload --upload-port COM5
   ```
4. Remove the IO0–GND wire and power-cycle again.

At 115200 baud the board prints its IP address at boot and a line each time a relay switches.

---

## Security

ArtSwitch is meant for closed show and lighting networks. There is **no login on the web page and no password on network firmware updates**: anyone who can reach the board on the network can switch the relays, change settings or flash new firmware. Don't connect it to an untrusted network, and keep it away from the internet.

---

## Troubleshooting

**Relay doesn't respond to Art-Net**
1. Open the web page, look at *Art-Net seen*.
2. Nothing listed → no data reaches the board. Check the controller's output is on (in MadMapper: the fixture is switched on), it sends to the board's IP or to the network's broadcast address, and it uses Art-Net, not sACN.
3. Listed, but another universe → set the matching universe on the board or the controller.
4. Right universe, relay still doesn't follow → check the base address, that the relay has a pin assigned, and its switch points: a value inside the dead zone doesn't change the relay.
5. Relay follows, but late → its minimum switch time is holding it back (countdown on the relay card).
6. Relay shows ON on the page but doesn't click → wiring: common ground, relay driven through a module or transistor, correct pin.

**`name.local` doesn't load, or still opens the old name after a rename**
Windows and browsers cache names for a minute or so. Wait, close the tab, or run `ipconfig /flushdns`. Use the IP address in the meantime.

**The "Name this board" pop-up keeps coming back / can't save settings or restart**
The board still has the default name `artswitch`. Give it a name in the pop-up.

**IP address changed a minute or two after power-up**
The board had taken its 169.254 fallback address and then a DHCP server answered within the 2-minute window. See [DHCP and the 169.254 fallback](#dhcp-and-the-169254-fallback).

**Relay clicked at power-up**
Probably on IO5, IO14 or IO15, which can pulse at boot. Move it to IO4, IO32 or IO33.

**USB flashing fails ("Failed to connect") with relays wired**
A relay input on IO2 that pulls the pin high stops the board from entering flash mode. Disconnect that relay while flashing, or move it to another pin.

---

## License

[MIT](LICENSE)
