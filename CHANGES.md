# Changes in this fork

This file lists the changes this fork makes on top of Bitcraze's AI-deck ESP32 (NINA-W102) firmware.

Based on: https://github.com/bitcraze/aideck-esp-firmware at `58f15fa5e14ec888f9ee965b6236aaf6d5c9e269` (tag `2025.02`).

## Bug fixes

**The GAP8 SPI link could stall for good under load.**
Under sustained streaming, traffic from the GAP8 stopped while WiFi stayed up, with the GAP8 holding
its ready (RTT) line high. The RTT interrupt woke `spi_task` through `xEventGroupSetBitsFromISR()`,
which fails silently when the FreeRTOS timer-task queue is full. It now uses a direct task
notification, and the idle wait re-checks the RTT line every 10 ms. (8ea4d8f)

**The WiFi radio died when lwIP used up the heap.**
Under sustained TCP streaming the radio stopped transmitting (unicast first, later beacons) while the
firmware ran on, after the free heap fell to a few KB. The 64 KB TCP send buffer and 64 dynamic WiFi
TX buffers let lwIP take nearly all of the ~44 KB of free heap, so the WiFi driver's own allocations
failed. Now `CONFIG_LWIP_TCP_SND_BUF_DEFAULT=8192` and `CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM=8`.
(8ea4d8f, 471a391)

**The WiFi radio stopped after about 450 streamed camera frames.**
With plenty of free heap, the radio stopped transmitting after roughly 450 frames (about 75 s) of
continuous streaming, and only a power cycle brought it back. The fault is in the closed-source WiFi
driver of ESP-IDF 4.3; the firmware now builds on ESP-IDF 5.4.1 (see Build and tooling), whose driver
does not show it. (11e3c75)

**A slow or vanished TCP host blocked all GAP8 traffic.**
`send()` could block indefinitely, and back-pressure through the full WiFi TX queue then stopped the
SPI link. Sends now time out after 1 s (`SO_SNDTIMEO`), partial writes are finished in a loop, and the
client is dropped after 5 s without progress; keepalive (5 s idle, 3 probes 2 s apart) and
`TCP_NODELAY` are set, close is serialized between the RX and TX tasks (mutex, `shutdown()` first),
and the host queues hold 4 packets instead of 2. (8ea4d8f)

**The TCP receive path trusted the length header.**
A length above the 1022-byte MTU overflowed the static receive buffer, and one below 2 underflowed the
CPX length to about 64 KB in `wifi_transport_receive()`, so any host on the network could corrupt
memory. Short reads and `recv()` errors desynced the stream, and a reset was retried forever, so the
single-client server never accepted another client. Reads now loop to completion, a bad length, EOF
or error closes the client, and `wifi_transport_receive()` also clamps. (8ea4d8f, ef348f6, 471a391)

**An unchecked SPI frame length from the GAP8 corrupted memory.**
A GAP8 app with a different SPI framing put the ESP into a silent reboot loop right after boot, before
the UART link to the Crazyflie came up, so the deck went quiet and GAP8 flashing, which goes through
this firmware, stalled. `spi_task()` used the GAP8's length unchecked and overran a static packet into
the FreeRTOS queues next to it; lengths below 2 underflowed. A non-zero length outside 2..1022 now
drops the frame and counts it (`spiRej` in the status log). (737b80f)

**A lost "client connected" notification left the GAP8 waiting.**
A TCP client connected but the GAP8 never started sending. `WIFI_CTRL_STATUS_CLIENT_CONNECTED` went
out once, at accept time, and can be lost on the way to the GAP8; a TCP client sends nothing that
would prompt a repeat. While a client is connected it is now re-sent to the GAP8 every second (the
STM32 still gets it only on a change), from its own packet instead of the shared `txp`.
(5a5035b, 471a391)

**Console forwarding misused its `va_list` and could stall the task that logged.**
`cpx_and_uart_vprintf()`, which also forwards each ESP log line to the STM32, reused a `va_list`
already consumed by `vprintf()` (undefined behaviour), wrote with unbounded `vsprintf()` into one
unlocked packet shared by all tasks, and waited for the router, so a task logging while the route to
the STM32 was backed up stalled. It now uses `va_copy()`, a truncating `vsnprintf()`, a mutex and the
new non-blocking `espAppSendToRouter()`, dropping the line when the queue is full. (471a391)

**The UART TX path copied two bytes past the payload.**
`uart_tx_task()` copied `payloadLength` bytes (data plus the 2-byte route) from a packet holding
`dataLength` bytes of data, which for a full 100-byte packet wrote one byte past the static TX packet.
It now copies `dataLength` bytes. (471a391)

**WiFi control packets could overflow the SSID and key buffers.**
`WIFI_CTRL_SET_SSID` and `WIFI_CTRL_SET_KEY` copied the payload into 50-byte buffers unchecked (an
empty packet underflowed the length) with the terminator one byte too far, so a shorter SSID after a
longer one kept a stale character, and `strncpy(dst, src, strlen(src))` could overflow the 32-byte
SSID field of `wifi_config_t`. SSIDs over 32 bytes and keys over 64 bytes are now ignored with a
warning, copies into `wifi_config_t` are bounded, and empty control packets are ignored.
(3bbc525, edf780f, 05ce143)

## Features

**TCP or UDP to the host, chosen at runtime.**
Send `WIFI_CTRL_SET_TRANSPORT` (0x21, `data[1]`: 0 = TCP, 1 = UDP) before `WIFI_CTRL_WIFI_CONNECT`;
TCP is the default, and the port 5000 socket is now opened at connect time. In UDP mode the host
announces itself with the 3-byte datagram `FER` (each one also re-sends "client connected" to the
GAP8); each datagram carries one CPX packet framed as on TCP, malformed or oversized datagrams are
dropped, and a failed send drops the packet instead of blocking. Adapted from
larics/aideck-esp-firmware-udp. (ef348f6, 5a5035b, 471a391)

**Access point channel.**
`WIFI_CTRL_SET_CHANNEL` (0x12, `data[1]` = 1..13), sent before `WIFI_CTRL_WIFI_CONNECT`, sets the AP
channel; other values are ignored. The default is channel 6 instead of 1. (8ea4d8f, ba0c69b)

**Deck name on the network.**
`WIFI_CTRL_SET_NAME` (0x13, payload `<hostname>\0<name>\0`, each under 64 bytes) sets the mDNS
hostname (`<hostname>.local`) and the `name` TXT item of the `_cpx._tcp` service; malformed payloads
are ignored. mDNS now starts before the router, so an early name is not overwritten by the default
`aideck-XXXXXX`. (3bbc525)

**Status log every 5 s.**
Always on, on the ESP console and so on the Crazyflie console: free and minimum heap, transport and
client state, rejected SPI frames (`spiRej`), WiFi TX queue depth and counters, and the SPI handshake
(`txn`, packets each way, GAP8 RTT level `gap_rtt`, `armed`). A `txn` that stops moving with
`gap_rtt 1 armed 1` means the GAP8 asked for a transfer and never clocked it.
(8ea4d8f, b4b795d, ef348f6, 737b80f)

## Build and tooling

**Ported to ESP-IDF 5.4.1.**
Build with `idf.py build` under ESP-IDF 5.4.1 (for example the `espressif/idf:v5.4.1` image); the app
runs on the 4.3 bootloader already on the deck. The port renames 5.x APIs, takes mDNS from the
vendored `espressif/mdns` managed component (`main/idf_component.yml`, `dependencies.lock`,
`managed_components/`) and keeps settings in `sdkconfig.defaults` (`CONFIG_FREERTOS_HZ=100` for the
code's raw tick counts; the 4.3 config is kept as `sdkconfig.idf43`). Settings not carried over take
5.4.1 defaults, for example a 2304-byte system event task stack instead of 5000. The GNU make build,
`tools/build` scripts, CI workflow and README still target ESP-IDF 4.3. (11e3c75)
