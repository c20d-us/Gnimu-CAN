# First car test

A checklist for the first complete unit (12V carrier v1.0 and OBD-II helper) in the car, the 2018 BMW F87 M2. It's two sessions: a **voltage log** first, with nothing transmitting, then the **firmware** itself. Each section ends with what its results decide.

Everything here is a plan, written before the boards arrived. Change it as you go.

---

## Before the car: on the bench

With the new board on the bench rig, before it goes anywhere near the car:

- [ ] **Board check.** `scripts/tool.sh board_check --flash --monitor`: the LED blinks, BLE advertises, and A0 reads.
- [ ] **Calibrate the supply reading.** Flash the firmware (`scripts/flash.sh --build --monitor`). Set the bench supply to about 12.6V and measure it **with a multimeter at the board's 12V input**, not the supply's display, which reads to only 0.1V. Read `Supply:` from the `D` report and set `kSupplyCalibration` in `Config.h` to *meter ÷ reported*, times the current value. Rebuild, flash, and check at 12.0V and 14.2V too: within about ±0.05V is good.
- [ ] **Sleep wake check on the new board.** Let it sleep (fake ECU `OFF`, 12.6V, 5 minutes), then wake it at 14.2V and look at the `D` report's `Sleep: this boot woke for …` line: compare *slept at* with *checks read*. The breadboard showed a 0.05–0.15V gap; the PCB's ground layout may shrink it. Set `kWakeCheckOffsetV` in `Config.h` to the gap (*checks read* minus *slept at*, with the supply unchanged throughout).
- [ ] **Fake ECU run.** Discovery, polling at about 25 Hz each, `OFF`/`ON`, and `GATEWAY`, as on the breadboard. Once the device has heard traffic by listening it makes no gateway probes for the rest of that boot, so for the gateway test, set the fake ECU to `GATEWAY` first, then restart the device (`R`).
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
| Car off, locked | Boot log, then `D` | `CAN: no traffic …`. A parked M2 sits above the 12.4V probe threshold (12.6V last time), so expect up to 5 gateway probes 30 s apart, then no more: the known trade-off. Note what each one reports |
| Ignition on | Boot or status line | `CAN: traffic at 500 kbit/s` (listening) **or** `CAN: gateway probe … an ECU answered` (gatewayed). Then `OBD: ECU 0x7E8 answered`, and which PIDs are supported. Vehicle: **ignition on** |
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

## After both sessions

- [ ] Update `Config.h` and `SystemConfig.h` from the results: probe threshold, engine-running level, parked hold, check interval, rise threshold.
- [ ] Answer open question 3 in `docs/architecture.md`, and add the M2's numbers to it.
- [ ] Decide whether the M2 profile comes next.
