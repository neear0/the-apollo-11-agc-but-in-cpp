# the apollo 11 agc but in c++

![luminary 099 flying a V49 attitude maneuver: the fdai ball turning, V06N18 on the dsky](docs/dsky.png)

## why the apollo 11 agc but in c++?

because I'm bored and I can recycle this as a college class project

## what it is

the original apollo 11 guidance computer source (`Luminary099` for the lunar module, `Comanche055` for the command module), assembled by a c++23 yul assembler and run on a c++23 block ii agc, inside a simulated lunar module, with a dsky you can actually press

- `agc/` - no ui
  - `agc::c_computer` - the cpu, timers, interrupts, i/o channels, and the restart monitor (night watchman, rupt lock, tc trap) and standby
  - `agc::c_dsky` - the display and keyboard
  - `yul::` - the assembler
  - `lm::` - the lunar module: imu (gimbals, cdus, coarse align, gyro torquing, pipas), 16 rcs jets, rigid body attitude, the crew's switches and hand controller
- `apollo11/` - the app: luminary 099 flying the lunar module, fdai ball and switches on the left, dsky on the right, drawn with [CelosUI](https://github.com/neear0/celosUI)
- `tools/agc_check/` - console tool: `assemble <MAIN.agc> --compare <rope>` and `run <MAIN.agc> "V16N65E"`

the assembler reproduces both original ropes word for word, checksums included. the autopilot in the rope is what flies the lunar module: the jets it fires turn the vehicle, the imu measures it, it reads that back

## build

- visual studio 2022, x64
- clone [CelosUI](https://github.com/neear0/celosUI) next to this repo (`..\CelosUI`), or set `CelosUIDir`. built and tested with CelosUI `d297d47`
- open `apollo11.sln`, build, run

## test

- `agc_check test` from the repo root: both ropes against the 1969 ones (`tests/ropes`), then luminary flying the lm: imu turn-on, V49, V48, hand flying, standby. about 5 seconds, exit code is the failures
- ci runs it on every push, without the ui (no CelosUI needed)

## try

- `V35E` - lamp test
- `V16N65E` - mission clock
- `V37E00E` - P00
- `RSET` - clear lights
- the imu turns on caged: 90 seconds with NO ATT lit, then the autopilot takes over
- `V37E00E` `V49E` `V25E` `+03000E` `+02000E` `+00000E` `PRO` `PRO` - automatic maneuver to those gimbal angles (30 and 20 degrees), watch the ball
- mode ATT HOLD and the arrows (pitch, roll) or `q` `e` (yaw) - fly it by hand, let go and it holds
- `V41N20E` and three angles - coarse align the imu, `V42E` and three angles - torque the gyros
- `V37E06E` then hold `PRO` - standby, hold it again to come back
- `V48E` then `PRO` `PRO` `PRO` - autopilot data (N46, N47), then the descent engine's trim (N48): its gimbal driven to the stop for a minute and back, V50N48 when done
- keyboard: digits, `v` `n` `+` `-` `c` `p` `r` `k`, enter

## not simulated

the descent and ascent engines (just the descent engine's trim gimbal), the radars, the aot, orbits and gravity, uplink. programs that want them will complain with alarms

the pad load, the erasable words the ground loaded before launch, is only what the programs here need (`agc/src/lm/pad_load.cpp`). the computer powers on with erasable cleared, so the first fresh start is alarm 1107 (PROG lit), as on a real cold start

comanche 055, the command module's program, assembles word for word but doesn't run: there's no command module

## credits

- the agc source: MIT instrumentation lab, 1969, public domain, transcribed by the [Virtual AGC](https://www.ibiblio.org/apollo/) project, via [chrislgarry/apollo-11](https://github.com/chrislgarry/apollo-11)
- yaYUL and yaAGC from Virtual AGC were the reference for how the assembler and cpu behave
