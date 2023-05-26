# LX RoCon -- tracking satellites and celestial objects

## Setup
- PiKRON LX RoCon, 2 axes in use, mode=8 (stepper motor without IRC)
- Synta (branded SkyWatcher) NEQ6 mount (kinematic equivalent of HEQ5)

## Conventions
- Ground--Axis "a"--Axis "b"--Sky (a=Az or Hour Angle, b=El or Dec)
- "a" connected as "A", "b" connected as "B"

# LX RoCon
## Manual commanding
### Comms I/O
- how to issue a command:
a) echo SOME:COMMAND > /dev/ttyACM0 # beware, ttyACM* suffix may vary in case of e.g. disconnecting while in useb) 
b) terminal program, e.g. minicom -D /dev/ttyACM0 # and then write one or more commands/queries interactively
- when using terminal, character echo is handy; if disabled by previous program, re-enable:
`echo 'ECHO:1' > /dev/ttyACM0`
- case matters, CapsLock is your friend

### Survival
- `echo STOP: > /dev/ttyACM0` stops motion, but motors kept powered, holding position
- `echo RELEASE: > /dev/ttyACM0` disconnect motors, no current, equivalent to disconnected cable
- **do not run axis specific commands before configuring the axes!**
Basic configuration: `./lxr_init.sh /dev/ttyACM0`
- help # (yes, lowercase) is an online help, you can also ask for help to a specific command, e.g. help AP

### Axis suffixes
- most of RoCon cmd/query commands need suffix designating axis: A,B,C,...
-- e.g. query position: command is "AP", to query position of axis B: APB?
-- or: set speed of axis A to 100: SPDB:100

## Homepage and docs
- https://www.pikron.com/pages/products/motion_control/lx_rocon.html
- https://gitlab.com/pikron/projects/lx_cpu/lx-rocon
- similar command set to earlier model MARS-8: http://cmp.felk.cvut.cz/~pisa/mars8/mars8_man_cz.html

## Misc
### Position units
- RoCon internally uses integer position units (APunit), in absolute range of signed 24-bit int, i.e. `-8388608 ... +8388607`
- in different motor controller type (settable on-the-fly via `REGMODEx, REGOUTMAP` cmds, this APunit may have different meaning. When controlling in closed-loop with IRC, it is equal to one IRC step (1/4 IRC period).
- however, in our case we use dq-control with stepper open-loop, `REGMODEx:8`. In that case, APunit may be thought as a micro-step, and the ratio of APunit to full step is configurable: *`REGPTIRCx` APunits = `REGPTPERx` full steps.*
-- see below the values used in our setup
### Velocity units
- there are two velocity commands: coarse `SPDx` (currently in use) and fine `SPDFGx`.
- `SPDx` takes an integer argument of speed `v`, range 16-bit signed (-32768..+32767), physical dimention is: *4000/256 APunit/s* (4kHz is internal RoCon's control loop samplerate).
-- example: `SPDA:1` sets axis A to revolve with velocity of 15.625 APunit/s

# Synta NEQ6, HEQ5
## Motor & gear ratios
- `#define NEQ5_MOTOR_STEPS    50` -- steppers are "1.8deg/step" which means using 4-phase stepping, i.e. 50 waveform periods per revolution
- `#define NEQ5_GEAR_RATIO    705` -- same value (705:1) for "HEQ5 Syntrek" & "NEQ6 Synscan"
-- HEQ Syntrek has gear ratio 47/9 and worm ratio 135/1
-- (N)EQ6 has gear ratio and 47/12 and worm ratio 180/1
- `#define LXR_USTEPS         200` -- the product `NEQ5_MOTOR_STEPS*NEQ5_GEAR_RATIO*LXR_USTEPS` yields APunit count per axis full circle. In this case *50*705*200 = 7050000 APunits/circle*. This value must be <2^24. I choose value <2^23 so we can do some wrapping without overflow.
-- it means we set `REGPTIRCx:200` and `REGPTPERx:1`
-- *WARNING!* In some cases it is possible to set ratio of REGPTIRCx/REGPTPERx so that the modulo-2^24 is equal to modulo full circle. In our case, however, the prime factors of Synta gear&worm ratio and bitwidth of RoCon's REGPTIRCx/REGPTPERx parameters do not allow to match these values.
- Synta gear ratios documented here: https://www.astroeq.co.uk/tutorials.php?link=ratios
### Position and velocity units
- With the gear ratio and chosen APunit granularity REGPTIRCx/REGPTPERx=200/1, the conversion units of angular position and velocity are:
-- APunit (argument of `APx?`, `Gx:`): 1 degree = 19583.33333.. APunits (1 arcsec = 5.44... APunits)
-- speed (`SPDx:`): `SPDx:1` is 0.00079787 deg/s, `SPDx:5000` =~ 4 deg/s

# SW tools
## TrackPV.java
Calls OreKit to calculate position+velocity vectors over specified time period
```
javac -classpath '/SOME_DIR/orekit/hipparchus-1.8-bin/*:/home/marek/orekit/orekit-10.3.jar' TrackPV.java
java -classpath '/home/marek/orekit/hipparchus-1.8-bin/*:/home/marek/orekit/orekit-10.3.jar:.' TrackPV 2023-05-25T20:00:00 720 > pv_iss_2023-05-25T200000_720.txt
```
## pv2joint.jl
Translates XYZ position vector to mount axes (topocentric frame with possibly tilted "a" axis)
```
julia> include("pv2joint.jl")
julia> writedlm("pv_iss_2023-05-25T200000_720.ab", th);
```

## lxrmount
Tracking 2-axis trajectory with RoCon & astro mount
- `gcc -ggdb -Wall -Wno-unused-variable -O -o lxrmount lxrmount.c -lm`
- `./lxrmount /dev/ttyACM0 pv_iss_2023-05-25T200000_720.ab`
