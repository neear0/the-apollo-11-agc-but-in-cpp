# the apollo 11 agc but in c++

![the dsky running luminary 099, V16N65 mission clock](docs/dsky.png)

## why the apollo 11 agc but in c++?

because I'm bored and I can recycle this as a college class project

## what it is

the original apollo 11 guidance computer source (`Luminary099` for the lunar module, `Comanche055` for the command module), assembled by a c++23 yul assembler and run on a c++23 block ii agc, with a dsky you can actually press

- `agc/` - the computer (`agc::c_computer`), the dsky (`agc::c_dsky`) and the assembler (`yul::`), no ui
- `apollo11/` - the app: boots luminary 099 behind a dsky drawn with [CelosUI](https://github.com/neear0/celosUI)
- `tools/agc_check/` - console tool: `assemble <MAIN.agc> --compare <rope>` and `run <MAIN.agc> "V16N65E"`

the assembler reproduces both original ropes word for word, checksums included

## build

- visual studio 2022, x64
- clone [CelosUI](https://github.com/neear0/celosUI) next to this repo (`..\CelosUI`), or set `CelosUIDir`
- open `apollo11.sln`, build, run

## try

- `V35E` - lamp test
- `V16N65E` - mission clock
- `V37E00E` - P00
- `RSET` - clear lights
- keyboard: digits, `v` `n` `+` `-` `c` `p` `r` `k`, enter

## not simulated

imu, radar, engines, standby, hardware restarts. programs that want them will complain with alarms

## credits

- the agc source: MIT instrumentation lab, 1969, public domain, transcribed by the [Virtual AGC](https://www.ibiblio.org/apollo/) project, via [chrislgarry/apollo-11](https://github.com/chrislgarry/apollo-11)
- yaYUL and yaAGC from Virtual AGC were the reference for how the assembler and cpu behave
