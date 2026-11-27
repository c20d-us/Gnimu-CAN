# First car test

A checklist for the first complete unit (12V carrier v1.0 and OBD-II helper) in the car, the 2018 BMW F87 M2. It's two sessions: a **voltage log** first, with nothing transmitting, then the **firmware** itself. Each section ends with what its results decide.

Everything here is a plan, written before the boards arrived. Change it as you go.

---

## Before the car: on the bench

With the new board on the bench rig, before it goes anywhere near the car:

- [ ] **Board check.** `scripts/tool.sh board_check --flash --monitor`: the LED blinks, BLE advertises, and A0 reads.
- [ ] **Calibrate the supply reading.** Flash the firmware (`scripts/flash.sh --build --monitor`). Set the bench supply to about 12.6V and measure it **with a multimeter at the board's 12V input**, not the supply's display, which reads to only 0.1V. Read `Supply:` from the `D` report and set `kSupplyCalibration` in `Config.h` to *meter ÷ reported*, times the current value. Rebuild, flash, and check at 12.0V and 14.2V too: within about ±0.05V is good.
- [ ] **Sleep wake check on the new board.** Let it sleep (fake ECU `OFF`, 12.6V, 5 minutes), then wake it at 14.2V and look at the `D` report's `Sleep: this boot woke for …` line: compare *slept at* with *checks read*. The breadboard showed a 0.05–0.15V gap; the PCB's ground layout may shrink it. Set `kWakeCheckOffsetV` in `Config.h` to the gap (*checks read* minus *slept at*, with the supply unchanged throughout).
- [ ] **Fake ECU run.** Discovery, polling at about 25 Hz each, `OFF`/`ON`, and `GATEWAY`, as on the breadboard, plus `SECOND` (a transmission ECU that answers first). With more channels turned on in `kObdPolls`, check the later bitmaps are read, then `SILENT xx` (the channel goes `NOT ANSWERING` and the rest keep their rates) and `REFUSE xx` (the channel is dropped). For the gateway test, set the fake ECU to `GATEWAY`: the device should probe within about 5 s of the bus going silent.
- [ ] **Termination switch off.** The CAN Pal's termination switch must be **off** before the board goes in the car: the car's bus is already terminated. (It's on for the bench rig.)
- [ ] **Flash the voltage logger last**, with the board on USB: `scripts/tool.sh voltage_log --flash --monitor`, then `CLEAR LOG` and `STATUS`.

Take: the unit, the OBD-II helper, a multimeter, the laptop and its USB-C cable, and a way to note times (a phone is fine).

---

## Session 1: voltage log

Nothing transmits in this session: the logger only listens. It records, once a second, the supply voltage (mean, min, max) and how many CAN frames the OBD port carries.

**Before you start:**

- [ ] Measure the battery at its posts with the multimeter (car off). Note it.
- [ ] Note the time you plug in: the log's `t_s` counts seconds from power-up.

**The drive** (times are a guide; note when each step actually happens, as minutes after plugging in):

| Step | How long | What it shows |
|---|---|---|
| Plug in with the car off and locked | 5 min | Parked voltage; whether the OBD bus is silent (gatewayed) when locked |
| Unlock, ignition on, engine off | 2 min | Whether the OBD bus wakes with the ignition |
| Start the engine, idle | 5 min | The cranking dip; the charging voltage after a start (the M2 read 15.1V) |
| Drive, including 10+ minutes of steady cruising | 15–20 min | **The low phase of intelligent alternator control**, which the 12.4V probe threshold and the RPM override are for. Lift off the throttle a few times: charging spikes should show in `v_max` |
| Stop, idle | 5 min | Warm idle voltage |
| Engine off, lock the car | 30–60 min | **How fast the surface charge decays**, which decides how soon voltage alone stops saying "engine running"; how long the bus stays awake after locking |

- [ ] Measure the battery at its posts again near the end of the drive, engine running, and note the time: that pairs a post reading with the log's pin-16 reading.

**Afterwards:** unplug, connect the unit to the laptop on USB only, and dump the log:

```bash
scripts/tool.sh voltage_log --dump m2-drive-1.csv
```

**What it decides:**

| Result | Decides |
|---|---|
| Frames per second while the car is on | **Listening or probing:** frames on the OBD port mean the device finds the bus by listening; none means the M2's port is gatewayed and needs the gateway probe |
| Frames when ignition-on but engine off | Whether ignition-on alone wakes the bus |
| Lowest cruising voltage | **The probe threshold (12.4V)** and whether the 13.2V engine-running level is ever reached while cruising |
| Voltage over the 30–60 min after shutdown | **The parked hold (5 min) and the 0.3V rise rule:** how long after shutdown the surface charge keeps the voltage above 13.2V, and how much it falls in the first minutes |
| Post reading vs logged reading | The voltage drop between the battery and OBD pin 16 |
| Any `# boot` sections after the first | Whether cranking browns the device out |

---

## Session 2: the firmware

The real firmware, with a laptop on the console and AutoXDL on the phone. **Drive with a passenger watching the laptop, or test parked**: don't drive while reading a screen.

**Setup:**

- [ ] On the bench, flash the firmware: `scripts/flash.sh --build`.
- [ ] In the car: plug the unit into the OBD-II port with **J3 off**, plug the laptop into it, then **J3 on**. (A USB-C laptop won't connect to a board that's already powered.)
- [ ] Record the console to a file for the whole session, so nothing is lost:

  ```bash
  script -q m2-firmware-1.txt scripts/tool.sh voltage_log --monitor
  ```

  (`tool.sh --monitor` opens the console only; the tool name doesn't matter.) Type `L` for the status line once a second.

**Checks:**

| When | Check | Look for |
|---|---|---|
| Car off, locked | Boot log, then `D` | `CAN: no traffic …`. A parked M2 sits above the 12.4V probe threshold (12.6V last time), so expect a gateway probe every 10 s until it sleeps, 5 minutes after parking: the known trade-off. Note what each one reports |
| Ignition on | Boot or status line | `CAN: traffic at 500 kbit/s` (listening) **or** `CAN: gateway probe … an ECU answered` (gatewayed). Then `OBD: answered: …` (every ECU that replied), `OBD: ECU 0x7E8 chosen`, and which PIDs are supported. Vehicle: **ignition on** |
| Engine idling | Status line | RPM and throttle values and **their rates** (target 25 Hz each); Vehicle: **engine running** |
| Driving | Status line | Rates holding; `D` for timeouts. Vehicle stays **engine running** even when the supply drops below 13.2V while cruising (the RPM override) |
| AutoXDL | Connect and record a few minutes | RPM, throttle and supply voltage in the recording |
| Engine off | Console | `OBD: no replies for 2 s - waiting for the vehicle`; Vehicle: **parked** once the supply falls below 13.2V |
| 5+ min after locking | Console | `Sleep: parked for 300 s … - sleeping`. The console then disconnects |
| Start the car again | LED, AutoXDL | The device wakes within about 30 s: from traffic, 13.2V, or a rise. AutoXDL reconnects (its auto-reconnect setting) |
| End | `D` | The job table (max µs and max late), `CAN:` error counters, `OBD:` timeouts |

**When you're done:** J3 off, or unplug the unit. Sleep isn't meant to look after the battery long-term.

**What it decides:**

| Result | Decides |
|---|---|
| Listening or probe | Confirms session 1; whether the gateway probe works on the real car |
| Supported PIDs | Which channels the M2 offers in Mode 01 (more channels next) |
| Reply rates while driving | **Whether polling needs more throughput:** multi-PID requests, a longer timeout, or the M2 profile reading identifiers directly |
| Timeouts | Whether the M2's ECU answers within 50 ms |
| Activity during cruising | Whether the RPM override holds the activity at "engine running" through the low-voltage phase |
| Time from shutdown to sleep, and from start to polling | Whether 5 min parked and 30 s checks suit the paddock |
| Job table | Whether the car's bus load changes anything (the bench's roughly 5 ms stall included) |

---

## Results: Hyundai Santa Cruz (2026-11-25)

The first in-car tests were on a Hyundai Santa Cruz rather than the M2: the firmware session only, parked and idling, with the console logged over USB (the device on USB first, then plugged into the OBD port, then switched on). They found four firmware problems (one suspected rather than seen), all fixed and retested the same evening.

**What the car's OBD port does**

- **Gatewayed, and quiet unless asked.** With the engine running, the port carries no traffic at all until a diagnostic request arrives, then only for a few seconds after the last one. While the device polls, it hears traffic; a few seconds after it stops, the port is silent again. So the device always finds this car by gateway probe, never by listening.
- **500 kbit/s, two ECUs answer** the functional request: the engine on `0x7E8` and the transmission on `0x7E9`. The engine supports RPM and throttle.
- **With the car off**, the gateway sometimes acknowledges a probe without anything answering it (`acknowledged, no reply`), and sometimes doesn't (`no acknowledgement`).
- **Supply at OBD pin 16:** 12.6–13.0V parked shortly after running, 14.2V idling, a dip to about 11.9V at cranking.
- **Polling:** RPM and throttle at 25 Hz each, no timeouts, one bus error in several thousand frames.

**What it found**

| Problem | Cause | Fix |
|---|---|---|
| Stuck in WaitingForVehicle after some switch-ons, fixed by a restart | Once the bus had been heard by listening, probes were off for the rest of the boot; this port falls silent a few seconds after polling stops | Probes need only silence at both rates, and go every 10 s with no per-boot cap |
| Sometimes chose an ECU that supports nothing polled (suspected; seen as two ECUs answering) | Discovery took the first reply, and which ECU answers first can vary | Discovery collects every reply for 50 ms and chooses `0x7E8` first (ISO 15765-4, python-OBD) |
| Found and polled the car on USB alone, switch off | CAN ran with no 12V | CAN runs only with a 12V supply |
| Went into the cutoff sleep at 14.2V just after a switch-on with USB connected | The supply level still said "below cutoff" from the 0V on USB | No sleep decision for 30 s after the supply arrives |

**Retest, after the fixes**

| Test | Result |
|---|---|
| Switch on, engine running, port quiet | Found by probe 4 s after switch-on |
| Switch off and on, 3 s gap | Found in 4 s |
| Switch off and on, 30 s gap | Found in 5 s |
| Three restarts (`R`) | Found each time, by the first probe |
| Engine off for about 45 s, switch on, then restarted | A probe every 10 s while off; found 4 s after the engine started |
| Parked 5 minutes, then the engine started | Slept after 5 minutes; woke on the supply rising 0.3V (5 checks, 30 s apart), found and polling |
| AutoXDL and Gnimu Monitor | Both connect and show data as expected |

**Still open**

- **The wake-check offset.** The first check read 12.90V against 12.68V at sleep, a 0.2V gap against the breadboard's 0.05–0.15V. The car's voltage was drifting after shutdown, so it isn't a clean measurement: the bench step above (*Sleep wake check on the new board*) still decides `kWakeCheckOffsetV`. The rise rule, which woke it here, measures from the lowest check reading, so the gap doesn't affect it.
- **Driving:** poll rates and the supply's low phase while cruising.
- **The M2**, which the sessions above were written for.

## Results: BMW M2 (2026-11-26)

The firmware session only, idling, with the console logged over USB.

- **The OBD port carries traffic** with the engine running, about 250 frames a second at 500 kbit/s, so the device found the bus by listening, with no gateway probe.
- **One ECU answers** the functional request: the engine on `0x7E8`, supporting RPM and throttle.
- **Requests sent straight to the engine (`0x7E0`) go unanswered**: every one timed out, so the device cycled between finding the ECU and losing it 2 s later. The gateway acknowledges them (the transmit error counter stayed near 0) but passes on only functional requests (`0x7DF`). Polling now switches to asking everyone after 3 unanswered direct requests. With that, the M2 polls RPM and throttle at the full 25 Hz each, steady over about a minute with no vehicle lost, and a client receives frames at 20 Hz.
- **Reply time:** with two channels, 13–15 ms average and 20–24 ms longest, measured in 10 ms `can` job steps, so the ECU and gateway together take about 10 ms. No timeouts after the switch in over 2,500 requests per channel.
- **Ten channels at about 69 requests a second:** every channel the M2 supports polled at its target (RPM 25 Hz, throttle 19–20, pedal 9–10, speed 10, the temperatures 1 Hz, ambient and fuel every 5 s), with no timeouts in about 3,300 requests and replies averaging 10 ms. The M2 supports every channel in the table except charge air cooler temperature (`0x77`). With replies arriving within one job step, the 10 ms `can` step (at most about 100 requests a second) is now nearer the limit than the ECU.
- **Values at idle:** coolant 81 °C, oil 66 °C, intake air 18 °C, ambient 12 °C, fuel 99.6 %, pedal 14.5 % with the foot off (the pedal PID reads a sensor, and doesn't start at 0 on every car).
- **The receive queue overflowed briefly** (`missed 13` in about 70 s). The port's broadcast traffic, about 250 frames a second, filled the 32-frame queue in about 130 ms when the `can` job was held up that long. The queue is now 128 frames, about half a second of that traffic.
- **Supply at OBD pin 16:** 15.0–15.1V idling shortly after a start.

---

## Results: both cars, one PID per request (2026-11-27)

The firmware at the `pre-bundling` tag (frames at 30 Hz, the 5 ms `can` job, the 128-frame receive queue, the BLE Console), idling, with D reports read over the BLE Console in Gnimu Monitor.

| | Santa Cruz | M2 |
|---|---|---|
| Requests to | `0x7E0` | `0x7DF`, everyone |
| Timeouts | 0 | 0 |
| Reply average / longest | 10 / 16 ms | 7 / 15 ms |
| Requests a second, all channels | about 66 | about 72 |
| RPM / throttle | about 22 / 17 Hz (targets 25 / 20) | about 23 / 18 Hz |

Both cars were at their single-PID limit: a round trip plus the wait for the next 5 ms step holds each to about 70 requests a second, just under what the table asks for. The Santa Cruz's engine doesn't list coolant (`0x05`) or oil temperature (`0x5C`). The BLE stack refused 2–3% of frames, and every job showed one stall of about 5–6 ms.

## Results: both cars, multi-PID polling (2026-11-27)

Several PIDs per request on a 20 ms cycle, RPM and throttle at 50 Hz, the 1 ms `can` job, and the support bitmaps in the log. Idling, then the engine switched off and on again, with D reports and event logs read over the BLE Console.

| | Santa Cruz | M2 |
|---|---|---|
| Multi-PID requests | 1632 in about 33 s | 937 in about 19 s |
| PIDs per request / busiest | 2.6 / 3 | 2.6 / 3 |
| Failed / late cycles / timeouts, engine running | 0 / 0 / 0 | 0 / 0 / 0 |
| RPM / throttle | 50 / 50 Hz | 50 / 50 Hz |
| Reply, one frame / several | about 10 / 19 ms (longest 25) | about 10 / 11 ms |

- **Multi-PID works on both.** RPM and throttle went from about 22 and 17 Hz to 50 Hz each, with fewer requests (about 49 a second, against about 70).
- **The M2's gateway passes flow control to `0x7E0`**, though it doesn't pass requests sent there: replies of several frames came back with flow control at the engine's own address, and it never switched to `0x7DF`.
- **The Santa Cruz is slow to answer flow control:** about 9 ms from it to the rest of the reply, so replies of several frames take about 19 ms, still within the 20 ms cycle.
- **Switching off made both fall back.** The ECU went quiet, the next three multi-PID requests timed out, and the poller blamed multi-PID: `multi-PID requests failed 3 times in a row (no reply in time) - one PID per request instead`, then `no replies for 2 s`. The fall back held for the boot, so both polled one PID per request after restarting, and the D report no longer said so. Fixed: after three timeouts it first asks for one PID alone, and falls back only if that's answered (see the next results for the rest).
- **The support bitmaps:** the Santa Cruz's engine lists `67` (engine coolant temperature, two sensors) but not `05`, and no ECU lists `5C`; its transmission lists only `01` and `20`. Coolant now comes from `0x67` on cars without `0x05`. Oil temperature isn't in Mode 01 on the Santa Cruz, and would need Hyundai's own identifiers. The M2 lists everything in the table but `77`, and also `68`.
- **The version showed `pre-bundling`** for a build with uncommitted changes on top. Builds from a changed tree now end in `-dirty`.
- **Fuel read 0.0 %** on the M2 just after its restart, from the readings taken while it was off and starting; worth a look next time.
- **The 5–6 ms stalls** show at a 1 ms step as skipped `can` runs (93–819 a boot): each one delays a request by up to about 6 ms. Not yet traced.

## Results: Santa Cruz, the switch-off check (2026-11-27)

Parked, then started, idled and switched off and on again, with the one-PID check, coolant from `0x67` and `-dirty` versions.

- **Coolant from `0x67` works:** `OBD: PID 67 Coolant supported`, reading 41 °C rising to 52 °C as the engine warmed.
- **Multi-PID while running:** 1026 requests, 2.6 PIDs each, RPM and throttle at 50 Hz, one failure in about 20 s, recovered.
- **The version** shows `pre-bundling-dirty`, and the D report kept the fall back in the next session.
- **Switching off still made it fall back.** The engine stopped at 116 s (`Vehicle: ignition on`); about 11 s later three multi-PID requests went unanswered, but RPM alone was answered, so it fell back (`no reply in time, though one PID alone is answered`); 2 s later the ECU went quiet altogether. Shutting down, the ECU stops answering multi-PID requests before single ones. Fixed: the one PID asked for alone is RPM, and an answer falls back only with the engine running (400 rpm or more); and the fall back, and flow control's move to `0x7DF`, now last a session rather than the boot, so each time the car is found it tries multi-PID again.

## Results: Santa Cruz, per-session fall back (2026-11-27)

The fall back lasting a session, the RPM check, one reply path for every request, and no flow control switch. Plugged in parked, started, idled, switched off until `Waiting for vehicle`, started again.

- **No fall back at the switch-off.** The engine stopped at 95 s, the ECU went quiet, and 2 s later `no replies for 2 s - waiting for the vehicle`, with no `multi-PID requests failed`. This time the ECU didn't stop answering multi-PID requests before single ones, so the engine-stopped check (`… with the engine stopped … - keeping them`) didn't come up; it's still to be seen on a car.
- **Multi-PID again after the restart:** the schedule lines, then 954 requests, 2.6 PIDs each, no failures, RPM and throttle at 50 Hz.
- **While running, both sessions:** about 3,000 multi-PID requests with no failures, no late cycles and no timeouts; coolant from `0x67`, 45–48 °C. Replies of one frame about 10 ms and of several about 20 ms (longest 26 ms), as before.
- **Waking:** after the switch-off, probes found an acknowledging but silent port at 113 s, no acknowledgement at 123 s (everything asleep), and an ECU answering at 133 s after the restart.

## Next run

- **On the M2:** the same switch-off and restart, and its fuel reading just after a restart.
- **Either car:** watch for `multi-PID requests unanswered with the engine stopped` at a switch-off, the check that hasn't come up yet.

---

## After both sessions

- [ ] Update `Config.h` and `SystemConfig.h` from the results: probe threshold, engine-running level, parked hold, check interval, rise threshold.
- [ ] Answer open question 3 in `docs/architecture.md`, and add the M2's numbers to it.
- [ ] Decide whether the M2 profile comes next.
