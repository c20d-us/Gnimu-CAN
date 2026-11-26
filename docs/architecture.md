# Architecture

This page describes how the Gnimu-CAN firmware is put together. Everything here is built and bench-tested except [vehicle profiles](#vehicle-profiles), which are still design. The [open questions](#open-questions) at the end still need answers.

Gnimu-CAN reads vehicle data from CAN, either directly or via OBD-II PIDs, and streams it over BLE to a third-party app as a custom BLE sensor. It follows the shape of the [Gnimu](https://github.com/c20d-us/Gnimu) firmware: the same layers, fixed-rate scheduler, run-state machine, boot log, console and diagnostics.

The design goal is the same as Gnimu's: **consistency**. The receiving app gets a packet at a fixed rate (e.g., 30 Hz) no matter how fast or slow the car answers.

---

## Vehicle scope

The first version is **vehicle-agnostic**. It uses only standard OBD-II over CAN (ISO 15765-4) and Mode 01 PIDs, so it should work on most cars without per-vehicle setup. Vehicle-specific variants come later as [vehicle profiles](#vehicle-profiles).

What "most cars" means here:

- **Covered:** cars that speak OBD-II over CAN. In the US that's every car from model year 2008, and many from 2003–2007.
- **Not covered:** older cars that use the pre-CAN OBD-II protocols (ISO 9141, KWP2000 on K-line, SAE J1850). This hardware has no transceiver for them.
- **v1 limits:** 11-bit IDs only, at 500 or 250 kbit/s. The standard also allows 29-bit IDs, which some vehicles use. That's a later addition if a target car needs it.
- **Per-car differences:** not every car supports every PID. The support check at ECU discovery drops what the car doesn't have, and says so in the log. RPM (`0x0C`) and throttle position (`0x11`) are among the most widely supported.

---

## Hardware

The first target is the **12V CAN carrier (XIAO) v1.0** from [Gnimu-PCB](https://github.com/c20d-us/Gnimu-PCB/tree/main/boards/gnimu-can), connected to the car through the OBD-II helper board.

| Part | Role |
|---|---|
| Seeed XIAO ESP32C6 | MCU. Its TWAI controller is the CAN controller. |
| Adafruit CAN Pal | CAN transceiver, powered from the XIAO's 3V3. |
| Pololu D24V7F5 | 12V to 5V buck regulator, feeding the XIAO's 5V pin through a Schottky diode. |
| OBD-II helper | Breaks out CAN_H, CAN_L, GND and +12V (OBD pin 16) from the J1962 plug. |

| XIAO pin | GPIO | Net | Use |
|---|---|---|---|
| D0/A0 | 0 | `V12_SENSE` | Switched 12V through a ÷11 divider (100kΩ + 10kΩ). 14.8V reads as 1.35V. |
| D8 | 19 | `CAN_SILENT` | CAN Pal S pin. R1 on the PCB pulls it high, so the transceiver is listen-only until the firmware drives it low. |
| D9 | 20 | `CAN_TX` | TWAI TX |
| D10 | 18 | `CAN_RX` | TWAI RX |
| — | 15 | user LED | The XIAO's single yellow LED, active low. |

What follows from the hardware:

- **The device is powered whenever the car's battery is connected.** OBD pin 16 is unswitched. The carrier's switch (J3) can cut power by hand, but the firmware has no way to cut its own. For long-term parking, J3 off or unplugging the device is the plan; the firmware's [sleep](#power-and-sleep) covers the shorter gaps, such as the paddock between runs.
- **Listen-only is the hardware default.** A blank or crashed MCU can't put anything on the car's bus. The firmware drives `CAN_SILENT` low only once it has confirmed the bit rate, or for the single frame of a [gateway probe](#bring-up).
- **The firmware can see the vehicle's supply voltage.** That provides evidence of whether the engine is running (alternator charging, about 13.5–14.8V) or the car is parked (about 12.0–12.7V), and warns it before it flattens the battery.
- **A USB console in the car needs a specific connection sequence.** The XIAO's 5V pin is the USB connector's VBUS line, so a device plugged into the OBD-II port with the switch on puts about 4.8V on it, and a USB-C host (such as a MacBook) won't attach to a VBUS that already has voltage present. To connect a laptop to a device plugged into the car's OBD-II port:
  1. Switch the device off with the carrier's power switch (J3).
  2. Plug the USB cable into the laptop and the device.
  3. Switch the device back on.

Plugging the laptop in before connecting the device to the OBD-II port works the same way. A USB-A port or hub on the laptop avoids the issue entirely. While both are connected, D2 keeps the laptop's 5V and the car's regulator apart.

The LiPo carrier has the same CAN section, with `VBAT_SENSE` (via ÷2 divider) on A0 instead. It would be a second hardware profile, as in Gnimu, with a battery power policy instead of the vehicle one. It's out of scope for v1 work, and may never get built.

A breadboard prototype runs the 12V carrier build unchanged. An adjustable bench supply stands in for the car's voltage, so the power logic sees real readings through the real divider (see [Bench rig](#bench-rig)).

---

## Layers

The same layers as Gnimu, plus `protocol/`, under `code/GnimuCAN/src/`, with the sketch and `Config.h` in `code/GnimuCAN/` as in Gnimu:

| Layer | What's in it |
|---|---|
| `core/` | Plain C++, host-tested: the scheduler, the run-state machine, the supply monitor, the bus search (listen windows, silence for gateway probes, starting over on a quiet bus), OBD discovery (every reply in P2, choosing the ECU, the backoff), the gateway probe policy and probe sequence, the OBD-II request/reply codec, the poller, vehicle activity, the sleep policy and wake check, the LED patterns, the report and status-line formatters, and the BLE JSON documents. |
| `platform/` | The board: clock, watchdog, reset reason, hardware ID, the TWAI controller and silent pin (`CanPort`), the supply ADC, the LED, the console port, deep sleep and the record kept across it (`DeepSleep`). ESP32-C6 only for now, but kept behind headers so `core/` stays testable. |
| `peripherals/` | The BLE transport: the Gnimu service on NimBLE, through the Arduino BLE library. |
| `protocol/` | The Gnimu frame encoder and layout, the protocol repo's C++ reference, copied in. |
| `app/` | The job table, the state handling, CAN bring-up and polling, the console, the diagnostics. |

These Gnimu pieces carried over nearly unchanged, copied, not shared: `Scheduler`, `ConsoleEntry`, `Report`, `TextOut`, `StatusLine`, `DeviceId`, `BuildInfo`, `ResetReason`, `RateTracker`, `StatusLed`, and the build script's warnings-as-errors check.

---

## Data flow

```
CAN bus ──► can job ──────► OBD poller ──────────► emit job (fixed rate) ──► BLE notify
            (drain RX,       (one request in         (Gnimu frame)
             send requests)   flight; latest reply,
                              its time and counts)
supply ADC ──► power job ──────────────────────────►┘
```

Collecting and emitting are independent:

- The **poller** runs whenever an ECU answers, whether or not an app is connected, so the latest values are already there when one subscribes.
- The **emitter** runs only while a client is subscribed. At a fixed rate it takes the latest values and sends one packet. It never waits on the bus.
- The frame's valid mask says which fields hold a value. The poller also keeps each channel's request, reply and timeout counts for console diagnostics.

### Emit rate

Gnimu-CAN has no GNSS, so under the protocol's [rate rule](https://github.com/c20d-us/Gnimu-protocol/blob/main/docs/frame.md#rate) it sends at a fixed rate of its own, and the app aligns its data to the app's GNSS source. (A device with GNSS sends one frame per GNSS solution instead; an all-in-one device with CAN would pace its frames by GNSS too.)

**20 Hz is a starting point, not a design limit.** The rate is one setting, `kEmitRateHz` in `Config.h`, and the `emit` job's period follows from it. The plan is to raise it as far as testing shows is worthwhile, or lower it if something can't keep up. Somewhere in the 30-50 Hz range is the target, above the Gnimu GNSS rates for the reasons below.

What sets the useful ceiling:

- **How fast the car answers.** Emitting faster than a channel updates only repeats values. The per-channel rates in the diagnostics show where that point is. With two channels sharing a typical car's 20–60 replies per second, it's roughly 10–30 Hz per channel. Getting more real data means raising the poll rate: fewer channels, multi-PID requests, or a vehicle profile that reads broadcast frames directly from the CAN bus instead of polling.
- **The BLE connection.** The consuming app sets the connection interval. iOS typically allows down to 15 ms, which is about 66 connection events per second. Going above roughly 50 Hz needs testing with a real phone and app.

How old the value in a row is has three parts:

- **Poll age:** how long ago the car answered. With a channel updating at *P* Hz, this averages about 1/(2*P*): 17–50 ms at 10–30 Hz.
- **Emit age:** how long ago the latest packet left the device. At emit rate *E*, it averages 1/(2*E*): 25 ms at 20 Hz, 20 ms at 25 Hz, 10 ms at 50 Hz.
- **BLE delivery:** packets arrive at the consuming device in the connection's events (every 15–30 ms or so), which adds a similar amount whatever *E* is.

So raising the emit rate shaves only a few milliseconds off the average age. The real reason to run **above** the GNSS rate is different. The device's clock and the GNSS receiver's clock aren't synchronised. At exactly the same nominal rate, the two drift past each other, and now and then a GNSS interval gets no new packet, so that row repeats the previous value. A margin of a few Hz makes that rare. The real gains in freshness come from polling the car faster.

The emitter keeps its rate fixed whatever happens upstream. That's the consistency goal: a higher rate is only worth it if it holds steady.

---

## CAN and OBD-II

### Bring-up

1. **Controller check.** At boot, TWAI is started once in listen-only mode, with the transceiver still held silent by R1, and stopped again. A failure here is a hardware fault.
2. **12V supply.** CAN runs only with a 12V supply (6V or more at the divider). On USB alone, the device is on the bench or being flashed, not in a car: the console, BLE and the supply reading all work, but the CAN controller stays off, and the status line and D report say `no 12V`. When 12V arrives, bring-up starts. If it goes away again (switched off with a laptop still plugged in), CAN stops, what was found is forgotten, and the device goes back to WaitingForVehicle. So the switch works as on/off even with USB connected.
3. **Bit-rate check.** The firmware listens for traffic at 500 kbit/s, then 250 kbit/s, in windows of up to 2 s alternating between the two. The first clean frame at the rate it's listening at confirms that rate, so a live bus is found within milliseconds. Bus errors mean traffic at the other rate, so it switches early, after 200 ms. This step only listens and never transmits.
4. **Go active.** It restarts TWAI in normal mode at the confirmed rate and drives `CAN_SILENT` low.
5. **ECU discovery.** It sends Mode 01 PID `0x00` to the functional address `0x7DF` and, as ISO 15765-4 (4.2.2) specifies for a scan tool, collects **every** reply from `0x7E8`–`0x7EF` that arrives within P2 (50 ms). Most cars have more than one ECU that answers, typically the engine on `0x7E8` and the transmission on `0x7E9`, and which answers first varies, so taking only the first reply could pick an ECU with nothing to poll. Each reply is that ECU's support bitmap for PIDs `0x01`–`0x20`.
6. **Choosing the ECU.** The way python-OBD finds the engine: the ECU on `0x7E8` (ISO 15765-4 recommends `0x7E0`/`0x7E8` for the engine) if it supports RPM or throttle; otherwise the ECU supporting RPM, then throttle, with the most PIDs supported in all. From then on, requests go to that ECU's physical address (response ID − 8, usually `0x7E0`), so only it answers. Any polled channel it doesn't list is dropped, and the log says so. v1's channels are all in the first bitmap's range; reading the later bitmaps (`0x20`, `0x40`, ...) comes with channels that need them. The fake ECU's `SECOND` mode puts a transmission ECU ahead of the engine on the bench.

Discovery is OBD-only (`core/ObdDiscovery.h`). The bus search (`core/BusSearch.h`) only finds the bus, says when it's silent enough for a probe, and notices when it goes quiet; it knows nothing of ECUs. A future CAN tap build, which only listens and decodes broadcast frames, would share the bus search and leave out discovery, probes, polling and every transmit path.

If no ECU answers discovery, or none that answers supports RPM or throttle, the device goes back to listening and asks again, less and less often: every 5 s for 3 tries, every 30 s for the next 3, then every 2 minutes. A car that has just switched on usually answers within the first few tries. A bus that chatters without ever answering OBD (modules awake in a parked car, or a car that answers only on 29-bit IDs) then sees a request every couple of minutes, not every few seconds, for as long as it chatters (about 35 an hour).

When ECUs answer but none supports a polled PID, the device logs which answered, shows it in the D report (`OBD: answered: 0x7E9 - none supports the polled PIDs, asking again later`), and stays in WaitingForVehicle, asking again on that backoff rather than polling. Polling would find nothing to ask for, declare the vehicle lost after 2 s and rediscover it, over and over.

Either way, **once the bus goes quiet**, no frames for 10 s, the device takes it that the car has switched off and starts over: the bit-rate check, and discovery from its first interval.

If step 3 hears nothing, there are two possible reasons: the car is off, or the OBD port is behind a gateway that stays silent until a scan tool asks for something. Gatewayed ports are common from roughly the late 2010s on, the BMW F-series included. The supply voltage is the only hint, and a weak one:

- **Supply below the probe threshold** (12.4V, set in `Config.h`): the car is treated as off. The device goes to [WaitingForVehicle](#run-states) and keeps listening.
- **Supply at or above it** for 3 s, with the bus silent at **both** rates (a whole listen window at each, with no frame and no bus error): the car may be on behind a gateway, so the device makes a **gateway probe**. Silence at both rates means a live bus at the other bit rate is always found first, so a probe never goes onto a bus running at the other rate.
  1. It drives `CAN_SILENT` low and starts TWAI in normal mode at the probe rate, 500 kbit/s (set in `Config.h`). Nearly every gatewayed car uses that rate.
  2. It sends one Mode 01 PID `0x00` request to `0x7DF` as a **single-shot** frame. The controller doesn't retransmit it, so a wrong rate produces at most one bad frame.
  3. A valid reply within 1 s confirms the rate, and bring-up continues at step 4. That's longer than the 50 ms ISO 15765-4 allows an ECU: a gateway passing the request on may answer later, and waiting costs nothing.
  4. Otherwise it stops TWAI, releases `CAN_SILENT`, logs the result ("no acknowledgement", "acknowledged, no reply" or "controller error"), and keeps listening in WaitingForVehicle.
  5. It probes again every 10 s for as long as the bus stays silent and the supply stays up. There's no cap, so a car started at any time is found: a parked car that never answers sees one frame every 10 s until the device sleeps, 5 minutes after parking, and wake checks only listen.
- **Having heard the bus earlier doesn't rule a probe out.** Some gatewayed ports carry traffic only for a few seconds after a diagnostic request, then fall silent until asked again (see the [car-test results](car-test.md#results-hyundai-santa-cruz-2026-11-25)). Silence at both rates is the only condition, since it's what keeps a probe off a live bus at the other rate.

Neither the probe nor discovery waits inside a job. Each sends its request and returns. Later `can` runs, every 10 ms, check whether the controller has reported the frame sent, take the replies as they arrive, or give up at the timeout. Until the step finishes, the rate check and probe policy stay paused. Waiting inside the job would hold BLE, the LED and the console for the whole timeout.

**Why 12.4V and not the 13.2V engine-running threshold.** Many modern cars, the BMW F-series among them, use intelligent alternator control: the alternator often stops charging while the engine runs, and the supply sits around 12.4–12.8V. A 13.2V trigger would miss those cars. The cost is that a healthy parked battery (about 12.6–12.8V) also passes, so a parked gatewayed car can see probes, one every 10 s until the device sleeps. A voltage log of a drive ([car-test.md](car-test.md)) will show whether it needs adjusting.

### Polling

- **One request in flight.** The poller sends a request and waits for its reply or for the 50 ms timeout (the ISO 15765-4 P2 limit) before sending the next. It never blocks: the `can` job runs every 10 ms, takes any reply, and sends the next request. That caps polling at about 100 requests per second, above what most cars answer; a faster job period is the change if a car ever outruns it.
- **A slow ECU.** One that answers later than the 50 ms timeout still gets polled: the device asks again, and a late reply that arrives while the same PID is in flight is taken as its answer, so data keeps coming at a few hertz rather than not at all. A 60 ms ECU gives about 3 Hz per channel, with three requests in four timing out, and is never counted as lost. ISO 15765-4 requires 50 ms, so this shouldn't happen on a real car; if one does answer slowly, a longer timeout (100 ms) would give it much more throughput.
- **Vehicle lost.** No reply on any channel for 2 seconds: the device logs it, goes back to listening, and returns to WaitingForVehicle. If the port then falls silent, probes start again as soon as it has been silent at both rates.
- **Throughput is shared.** A car typically answers 20–60 single-PID requests per second. Every polled PID takes a share of that, so 20 Hz of real data for every channel isn't possible on most cars.
- **Tiered schedule.** Each channel has a target rate, and the poller picks whichever channel is most overdue relative to its own target. v1's two channels both aim for 25 Hz. As channels are added, fast ones such as RPM, throttle and speed get most of the budget, and slow ones such as temperatures about 1 Hz. The status line shows the rate each one actually gets.
- **Bus-off.** Not handled on its own: a controller that goes bus-off stops getting replies, so after 2 s the vehicle-lost path restarts it in listen-only mode.
- **Single-frame replies only, for v1.** Every Mode 01 PID fits in one frame. Asking for several PIDs per request raises throughput a lot, but the replies span several frames and need ISO-TP flow control. That's a later step, and the poller's interface leaves room for it.

### Channels

The first build sends **RPM and throttle position**, replacing what an ELM327-style OBD-II dongle (such as the Veepeak OBDCheck BLE+) gives AutoXDL today. More channels come later. Each one already has its field in the [Gnimu frame](https://github.com/c20d-us/Gnimu-protocol/blob/main/docs/frame.md), so adding one never changes the frame.

| Channel | PID | Decode | Frame field | In v1 |
|---|---|---|---|---|
| RPM | `0x0C` | (256A + B) / 4 | Engine speed (vehicle bit 0) | Yes |
| Throttle position | `0x11` | A × 100 / 255 % | Throttle position (vehicle bit 1) | Yes |
| Supply voltage | ADC, divider × 11 | — | Supply voltage (power bit 0) | Yes, from the ADC, not the bus |
| Coolant temperature | `0x05` | A − 40 °C | Coolant temperature (vehicle bit 4) | Later |
| Intake air temperature | `0x0F` | A − 40 °C | Intake air temperature (vehicle bit 5) | Later |
| Ambient air temperature | `0x46` | A − 40 °C | Ambient air temperature (vehicle bit 6) | Later |

Each channel's target period is in `Config.h` (40 ms, 25 Hz, for both to start). The channel list itself, PID, decode and frame field, is in `App.cpp` for v1; it becomes a table when vehicle profiles arrive. A value older than 1 second is left out of the frame (its valid bit clear) rather than sent stale. With two channels, each gets about half of the car's reply budget: roughly 10–30 Hz per channel on a typical car, which is the number the diagnostics will confirm.

### Vehicle profiles

*Design, not built yet.* Everything vehicle-specific sits behind a **vehicle profile**, chosen at build time the way Gnimu chooses its hardware and protocol profiles. v1 has one profile, `obd2`. A profile supplies:

- **How to reach the car:** bit rate, 11- or 29-bit IDs, request and response IDs, and the addressing scheme.
- **How to ask for each channel:** the service and identifier (Mode 01 PID, or a manufacturer-specific identifier), and how to decode the reply.
- **How to tell the car is awake,** for the bring-up and power logic.

The scheduler, poller, emitter, BLE packet, power and sleep logic, console and states don't change between profiles. A channel is a channel, wherever it came from.

What a manufacturer-specific profile is likely to need, and v1 leaves room for:

- **UDS `0x22` ReadDataByIdentifier** requests to a specific ECU, instead of Mode 01. This is how most manufacturers expose data beyond the standard PIDs.
- **ISO-TP multi-frame replies with flow control,** because manufacturer identifiers often return more than one frame of data.
- **Extended addressing,** where the target ECU's address goes in the first data byte. BMW's diagnostic CAN works this way, so a BMW F-series profile (such as the F87 M2) would need it.
- **Diagnostic sessions and tester-present keep-alives,** for identifiers that are only readable in an extended session.

The `obd2` profile also stays useful on those cars: Mode 01 PIDs answer on BMWs too, so the generic build works on the M2 before its own profile exists.

---

## BLE

The device advertises as `Gnimu-CAN` followed by the 10-digit device ID, the same scheme as Gnimu, with the service UUID in the advertising data so AutoXDL's scan finds it. It sends only the 140-byte [Gnimu frame](https://github.com/c20d-us/Gnimu-protocol/blob/main/docs/frame.md), the layout shared by the whole Gnimu family, at the [emit rate](#emit-rate) (20 Hz to start) while a client is subscribed, to one client at a time. It fills the header, the vehicle fields it polls and the supply voltage, and sends the GNSS and IMU blocks as zeros. The encoder is the protocol repo's C++ reference, copied into `src/`. The UUIDs, the fields it fills and the AutoXDL setup are in [ble-protocol.md](ble-protocol.md).

The console's `P` command prints the protocol setup (service, notify characteristic, packet length, and each filled field's type, offset and scale), so setting up an app means copying it from the console.

---

## Run states

| State | Meaning | Jobs running |
|---|---|---|
| **Starting** | Boot: supply reading, CAN bring-up, BLE start. | None yet |
| **WaitingForVehicle** | Hardware is fine, but no ECU is answering yet: the bus is silent, or discovery is under way or being retried. | All |
| **Polling** | An ECU answers, and the vehicle fields are being polled. Streams when a client is subscribed. | All |
| **Fault** | The CAN controller or BLE failed to start. Doesn't retry; `R` restarts. | `power`, `led`, `console` |
| **Sleep** | Car parked or the battery low, so the device goes into deep sleep with a timer wake. Like a restart, it leaves the run; waking is a fresh boot. | None |

| From | Event | To |
|---|---|---|
| Starting | Hardware failed | Fault |
| Starting | Hardware fine | WaitingForVehicle |
| WaitingForVehicle | ECU found | Polling |
| Polling | No replies for 2 s | WaitingForVehicle |
| Any running state | Parked for 5 minutes, or below the cutoff ([Sleep](#sleep-and-the-cutoff)) | Sleep |
| Sleep | Timer wake that finds the car awake | Starting (fresh boot) |

Starting always goes through WaitingForVehicle, even when traffic is heard at boot: discovery runs in the `can` job, so the device is waiting until the ECU answers, a few milliseconds later.

The state is called Polling, not Running, because it's about whether the device can talk to the car, not whether the engine runs: an ECU answers with the ignition on and the engine off too. Whether the engine runs is the separate [vehicle activity](#vehicle-activity). The frame header's state code for it stays "running" (2), since that code describes the device.

Unlike Gnimu, Polling and WaitingForVehicle move back and forth without a restart. A car switching off and on again is normal operation, not a failure. Sleep, though, always comes back through a fresh boot, so the device still starts up only one way.

While waiting, a subscribed client still gets packets at the emit rate, so the app's connection stays up. The vehicle fields read 0 with their valid bits clear, the header's state says waiting, and the supply voltage stays live.

---

## Power and sleep

Because the device runs from OBD pin 16, it has to stop drawing current on its own when the car is parked.

### Vehicle activity

Separate from the run state (can the device talk to the car?), the device keeps track of what the car is doing, from the best signal it has:

| Signal available | Activity |
|---|---|
| **RPM, fresh** (answered within the last second) | **Engine running** at 400 RPM or more, past cranking speed. **Ignition on** below that: an ECU answers, but the engine isn't turning. |
| **No fresh RPM** (bus silent, or no ECU) | **Engine running** with the supply at 13.2V or more for 3 s. **Parked** below that, or below the cutoff. |
| Neither (no supply reading yet) | **Unknown** |

The console logs each change as `Vehicle: …`, and the status line and `D` report show it. RPM overrides the supply voltage whenever it's fresh. That matters on cars with intelligent alternator control, whose supply can sit at 12.4–12.8V with the engine running: voltage alone would call that parked. Voltage is the fallback for when the device can't reach an ECU, which includes every parked car.

### Sleep and the cutoff

The device goes into ESP32-C6 deep sleep with a timer wake. Each wake runs a quick check and either goes straight back to sleep or carries on into a full boot. The times and voltages below are placeholders in `Config.h` until the car's voltage through a drive and after shutdown has been logged.

**When it sleeps.** The power job decides, every 100 ms, in every state:

- **Parked:** no fresh RPM, the supply below the engine-running level (13.2V), and no CAN frame for 10 s, all held continuously for 5 minutes. A connected client doesn't keep the device awake: a phone left in the car would otherwise hold it at full current indefinitely.
- **Low-voltage cutoff:** the supply below 11.8V for 5 s, timed by the sleep logic itself so it applies straight after boot too (a boot during a cranking dip, or while a supply ramps up, doesn't sleep on its first reading). It sleeps without the 5-minute hold or a silent bus, unless fresh RPM says the engine is running: with the engine running, the device's few milliamps don't matter next to the car's own load (a failing alternator, say), and logging continues.
- **Not in the first 30 s after the supply arrives**, at power-up or when switched on with USB already connected. The supply level lags the voltage by its hold times, so after 0V on USB it can still say "below cutoff" for a few seconds at 14V, and the car's voltage may still be ramping.
- **Never on USB power alone.** Below 6V the regulator can't run the XIAO, so the device must be on USB with no 12V connected: there's no battery to protect, and it stays awake for a bench session or a flash. A wake check that reads below 6V boots fully, and the D report says `Sleep: off - supply below 6V, running from USB`.
- **Not while `W` is on.** The console's `W` keeps the device awake until `W` again or a restart, so a bench session with a laptop attached isn't interrupted. It's off at every boot.

Going to sleep logs a `Sleep:` line, stops the CAN controller (the transceiver stays silent), turns off the LED and BLE, and sleeps for the check interval: **30 s** when parked, **10 minutes** after the cutoff.

**The wake check.** Every timer wake is a fresh boot, but before the console, BLE or the boot log, it takes an averaged supply reading and listens on the bus for 150 ms at 500 kbit/s and then 250 kbit/s. It never transmits, so a parked car sees nothing from the device, and a gatewayed car can't be kept awake by probes. It boots fully if any of these hold:

- **CAN traffic** was heard.
- **The supply is at or above 13.2V.**
- **The supply has risen 0.3V or more above the lowest wake-check reading since the device went to sleep:** the alternator has started charging. This catches a gatewayed car with intelligent alternator control, which can run at 12.4–12.8V with a silent OBD bus and so look parked. The baseline is the lowest reading, so a battery's surface charge decaying after a drive never looks like a rise.
- **The supply is below 6V:** no 12V, so the device is on USB (see above).

Otherwise it sleeps again. A full boot after a wake logs `Sleep: woke after N checks …` with the reason, and from then on behaves like any boot. That includes gateway probes: a probe is useful exactly when a check has seen signs the car is awake, as with a gatewayed car whose supply has risen.

**A check reads higher than the awake device.** On the bench, a check read 0.05–0.15V above the awake reading at the same supply, most likely because the awake device draws more (BLE on), which shifts the reading. Two things allow for it:

- **The rise rule compares check readings only.** The first check after going to sleep sets the baseline; the awake reading isn't used. The cost: a car started before the first check, within 30 s of sleeping, can wake the device only by reaching 13.2V, not by the rise rule.
- **The check's other levels are raised by `kWakeCheckOffsetV`** (0.10V to start, in `Config.h`), so 13.2V, the 11.8V cutoff and the 12.1V recovery mean the same voltages asleep as awake. The offset is per board: with the supply unchanged, it's the D report's "checks read" minus "slept at".

**The cutoff's hysteresis.** Asleep for the cutoff, the device checks every 10 minutes and wakes fully only for traffic or 13.2V. It goes back to the 30 s parked interval only once the supply is above 12.1V, so a battery hovering at the cutoff, with ADC noise of about ±0.1V, can't flip it between the two. A parked device whose battery falls below 11.8V moves to the cutoff interval at its next check.

- CAN RX (GPIO18) isn't one of the C6's low-power GPIOs, so CAN traffic can't wake the chip directly. The timer wake is the only way back.
- Sleep isn't a [run state](#run-states) the device stays in: like a restart, it leaves the run, and waking is a fresh boot.

**Long-term parking is J3 off, or the device unplugged.** Sleep keeps the draw down between drives and runs, but it isn't relied on to look after a battery for weeks, so the sleep current (regulator, divider, XIAO deep sleep, and the CAN Pal, whose supply stays on) hasn't been measured and doesn't set any of the values above. Measuring it on the real board is optional; it would matter only if the device were ever meant to stay plugged in and switched on while parked for long periods.

---

## Scheduler jobs

| Job | Every | Work |
|---|---|---|
| `can` | 10 ms | Drains TWAI RX. Waiting: checks the bit rate, makes gateway probes, runs ECU discovery. Polling: matches replies, times out the request in flight, sends the next, and notices the car going quiet. |
| `emit` | 1 / emit rate (50 ms at 20 Hz) | Encodes a Gnimu frame from the latest values and notifies, if a client is subscribed. |
| `power` | 100 ms | Reads the supply ADC, filters it, feeds the probe policy, logs supply and vehicle-activity changes, and decides when to [sleep](#sleep-and-the-cutoff). |
| `ble` | 10 ms | BLE housekeeping. |
| `led` | 25 ms | Status LED. |
| `console` | 10 ms | Commands and the status line. |

Table order is priority, as in Gnimu: draining the bus comes first.

---

## Console and diagnostics

Same port, baud and commands as Gnimu: `D` report, `C` clear counters, `V` version and config, `L` status line, a number for N status lines, `R` restart. Plus `P`, which prints the BLE protocol setup, and `W`, which keeps the device awake (no sleep) until `W` again or a restart.

**Console output never blocks.** The console is the C6's built-in USB serial port. With a cable attached but no terminal reading, its transmit buffer fills, and by default every write then waits 100 ms for room, which stalls the whole loop: polling, BLE frames, everything. The firmware sets the port's transmit timeout to 0 (`Serial.setTxTimeoutMs(0)`), so output nobody is reading is dropped instead.

Boot log in the same style:

```
BUILD,version=0.1.0,revision=...,utc=...,target=esp32c6
Device ID: 0301821570
Config: hardware 12v, vehicle obd2, CAN auto 500/250 kbit/s, gateway probe at 12.40V
Config: BLE "Gnimu-CAN 0301821570", Gnimu frame v1, 140 bytes at 20 Hz
Config: supply calibration 1.028, engine running 13.20V, cutoff 11.80V
Gnimu-CAN starting
Reset: power-on
Watchdog: 5000 ms ✅
Supply: 14.20V, engine running
CAN: controller started, listen-only ✅
CAN: traffic at 500 kbit/s ✅
BLE: advertising as "Gnimu-CAN 0301821570" ✅
Job can: every 10000 us, budget 2000 us
...
Waiting for vehicle.
Commands end with Enter: ...
OBD: answered: 0x7E8, 0x7E9
OBD: ECU 0x7E8 chosen, polling via 0x7E0 ✅
OBD: PID 0C RPM supported ✅
OBD: PID 11 Throttle supported ✅
Vehicle: engine running
```

The `D` report adds CAN and OBD lines to Gnimu's job table: bus state and error counters, the ECU being polled and every ECU that answered discovery, and for each channel its latest value with its request, reply and timeout counts, or that the ECU supports none of the polled PIDs, then the [vehicle activity](#vehicle-activity), and a `Sleep:` line: whether the parked conditions hold and for how long, that `W` is keeping it awake, or that it's on USB alone, and how many times it has slept since power-up. After a wake, a second `Sleep:` line gives the reason, the number of checks, the voltage it went to sleep at and the range the checks read. The status line shows the state, the supply voltage, `CAN: off, no 12V` on USB alone, each channel's value and reply rate (replies per second since the previous line), the vehicle activity, and the subscribed client's packet rate.

The status LED is the XIAO's single yellow LED, so it shows patterns only, like Gnimu's ESP32 build:

| Pattern | Meaning |
|---|---|
| A short flash every 2 s | WaitingForVehicle |
| Slow blink (1 s on, 1 s off) | Polling, no client |
| Solid | Polling, client connected |
| Fast blink | Fault |

---

## Testing

- **Host tests** (`tests/run.sh`) for `core/`: the scheduler, rate tracking, device ID, console input, the supply monitor, the state machine, the LED patterns, the report, status-line and JSON text, OBD request encoding and reply parsing (including the malformed and wrong-ECU cases), the probe policy and two-frame probe, the bus search (including the bench-found probe cases), discovery (including the transmission answering first), the poller (one in flight, timeouts, support bitmap, rate shares), vehicle activity, and the sleep policy and wake check. Each runs plain and under AddressSanitizer and UndefinedBehaviorSanitizer.
- **Build checks** with warnings as errors in the project's own code, as in Gnimu.
- **On hardware**: everything on the [bench rig](#bench-rig), then the car itself: the bit-rate check or gateway probe, poll throughput, sleep and wake timings, and timing under load from the diagnostics job table ([car-test.md](car-test.md)).

### Bench rig

Two breadboards, joined by a CAN_H/CAN_L/GND pair:

- **Device:** a XIAO ESP32C6 and a CAN Pal wired as on the 12V carrier, with R1 on `CAN_SILENT` and the ÷11 divider on A0. An adjustable bench supply (0–30V) on the 12V input powers it through the buck regulator and plays the car's voltage: about 12.4V parked, about 14.2V engine running, below 11.8V for the low-voltage cutoff, and anything in between to test the transitions. Set its current limit to about 150–200 mA. Keep below about 16V if the TVS diode (D1) is fitted, since it starts conducting around 17V. Keep the Schottky diode (D2) between the regulator and the XIAO's 5V pin: the USB cable for the console will be plugged in at the same time.

  **Connect USB before turning the supply on.** The XIAO's 5V pin is the USB connector's VBUS line, so with the supply on, about 4.8V sits on VBUS before a cable is attached. A USB-C host (such as a MacBook) checks that VBUS is at 0V before it attaches, sees the voltage, and doesn't enumerate the board. In the other order it attaches normally and stays attached. A USB-A port or hub doesn't make that check. The same applies to the 12V carrier if a USB-C cable is plugged into a board already running from the car. CAN runs only once the supply is on: on USB alone the device keeps CAN off, so a fake ECU test needs the bench supply.
- **Fake ECU:** a second XIAO ESP32C6 and CAN Pal running a small sketch (`tools/fake_ecu/`) that answers Mode 01 requests on `0x7E8` with changing RPM and throttle values. It has settings for a reply delay, so it can stand in for slow or fast ECUs, for staying silent until asked, like a gatewayed port, for a transmission ECU that answers first without RPM or throttle (`SECOND`), for supporting neither RPM nor throttle (`HIDE`), and for chattering without answering OBD at all (`MUTE`). If its controller goes bus-off (it can, when its bit rate is changed while the device is still active at the old one), it restarts it by itself. It runs from the USB-C or from the 5V supply from the Gnimu-CAN breadboard that has an in-line Schottky diode (a 1N5819, like D2), so it's safe to have its USB cable plugged in too. It has no voltage divider or buck regulator.
- **Termination:** both CAN Pals' termination switches **on** for the bench, so the bus has a 120Ω end on each side as a car does. Both must be **off** before either is used in a car.

Without the fake ECU, the device alone still exercises boot, states, voltage sense, BLE and, with the supply at or above the 12.4V probe threshold, the gateway probe's failure path (no node to acknowledge it). With it, the whole OBD path can be tested: discovery, the gateway probe's success path, polling rates and the diagnostics.

---

## Open questions

1. **Sleep thresholds and wake interval.** Tuned from the car's logged voltage, through a drive and after shutdown (question 3). Not from the sleep current: long-term parking is J3 off or the device unplugged.
2. **Gateway probes on a timer wake.** *Settled:* a wake check only listens. A full boot, which can probe, follows only when the check sees traffic, 13.2V, or a 0.3V rise since sleeping. A parked M2 at 12.6V sees no frames from a sleeping device.
3. **Supply voltage through a drive and after shutdown.** The M2's first readings, at the battery posts, were 12.6V parked and 15.1–15.15V in the first 4 minutes after a cold start: the charging phase. Still needed: the low phase of intelligent alternator control while cruising, which the 12.4V probe threshold and the RPM override are built for, and the surface-charge decay after switching off, which decides how soon voltage alone stops saying "engine running". The device's own status line (`L`) can log both on the first car test. Its reading is at OBD pin 16, after the car's wiring and fuse, so it may sit slightly below the battery-post figures.
