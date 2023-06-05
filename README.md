# LX RoCon -- tracking satellites and celestial objects

## Why
- some mounts such as our and popular Synta (SkyWatcher) HEQ5, (N)EQ6 Syntrack/SynScan do have reasonable gears and stepper motors capable to reach satellite tracking speeds (3--4deg/s in default factory slewing speed), however they are **impaired by a totally stupid** PIC stepper driving firmware, not allowing to smoothly adapt speed without interruption
  - there are other mounts not having this problem, yes, but:
- ...it may be handy to just have a generic code base able to track sats with a mount + two stepper motors, no absolute encoders, no feedback
- so we decided to take the industrial quality PiKRON RoCon controller and attached it to the COTS mount. First successful satellite tracked visually was H-2A at 2023-06-04T21:43:00 via Rubinar 500mm f/5.6 + 25mm eyepiece.

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
  - a) `echo SOME:COMMAND > /dev/ttyACM0` beware, `ttyACM*` suffix may vary in case of e.g. disconnecting while in useb) 
  - b) terminal program, e.g. `minicom -D /dev/ttyACM0` and then write one or more commands/queries interactively
- when using terminal, character echo is handy; if disabled by previous program, re-enable:
`echo 'ECHO:1' > /dev/ttyACM0`
- case matters, CapsLock is your friend
- note: USB is not officially supported for real operation, use RS232 or Ethernet&TCP/IP instead. For the real operation we use `lxrmount` via TCP/IP.

### Survival
- `echo STOP: > /dev/ttyACM0` stops motion, but motors kept powered, holding position
- `echo RELEASE: > /dev/ttyACM0` disconnect motors, no current, equivalent to disconnected cable
- **do not run axis specific commands before configuring the axes!**
Basic configuration: `./lxr_init.sh /dev/ttyACM0`
- help # (yes, lowercase) is an online help, you can also ask for help to a specific command, e.g. help AP

### Axis suffixes
- most of RoCon cmd/query commands need suffix designating axis: A,B,C,...
  - e.g. query position: command is "APx", to query position of axis B: `APB?`
  - or: set speed (command "SPDx") of axis A to 100: `SPDB:100`

## Homepage and docs
- https://www.pikron.com/pages/products/motion_control/lx_rocon.html
- https://gitlab.com/pikron/projects/lx_cpu/lx-rocon
- similar command set to earlier model MARS-8: http://cmp.felk.cvut.cz/~pisa/mars8/mars8_man_cz.html
  - English manual of even older unit, yet with mostly the same command semantics: https://cmp.felk.cvut.cz/~pisa/mars/mars_man_en.html

## Misc
### Position units
- RoCon internally uses integer position units (APunit), in absolute range of signed 24-bit int, i.e. `-8388608 ... +8388607`
- in different motor controller type (settable on-the-fly via `REGMODEx, REGOUTMAP` cmds, this APunit may have different meaning. When controlling in closed-loop with IRC, it is equal to one IRC step (1/4 IRC period).
- however, in our case we use dq-control with stepper open-loop, `REGMODEx:8`. In that case, APunit may be thought as a micro-step, and the ratio of APunit to full step is configurable: *`REGPTIRCx` APunits = `REGPTPERx` full steps.*
  - see below the values used in our setup
### Velocity units
- there are two velocity commands: coarse `SPDx` (currently in use) and fine `SPDFGx`.
- `SPDx` takes an integer argument of speed `v`, range 16-bit signed (-32768..+32767), physical dimention is: *4000/256 APunit/s* (4kHz is internal RoCon's control loop samplerate).
  - example: `SPDA:1` sets axis A to revolve with velocity of 15.625 APunit/s

# Synta NEQ6, HEQ5
## Motor & gear ratios
- `#define NEQ5_MOTOR_STEPS    50` -- steppers are "1.8deg/step" which means using 4-phase stepping, i.e. 50 waveform periods per revolution
- `#define NEQ5_GEAR_RATIO    705` -- same value (705:1) for "HEQ5 Syntrek" & "NEQ6 Synscan"
  - HEQ Syntrek has gear ratio 47/9 and worm ratio 135/1
  - (N)EQ6 has gear ratio and 47/12 and worm ratio 180/1
- `#define LXR_USTEPS         200` -- the product `NEQ5_MOTOR_STEPS*NEQ5_GEAR_RATIO*LXR_USTEPS` yields APunit count per axis full circle. In this case *50*705*200 = 7050000 APunits/circle*. This value must be <2^24. I choose value <2^23 so we can do some wrapping without overflow.
  - it means we set `REGPTIRCx:200` and `REGPTPERx:1`
  - *WARNING!* In some cases it is possible to set ratio of REGPTIRCx/REGPTPERx so that the modulo-2^24 is equal to modulo full circle. In our case, however, the prime factors of Synta gear&worm ratio and bitwidth of RoCon's REGPTIRCx/REGPTPERx parameters do not allow to match these values.
- Synta gear ratios documented here: https://www.astroeq.co.uk/tutorials.php?link=ratios
### Position and velocity units
- With the gear ratio and chosen APunit granularity REGPTIRCx/REGPTPERx=200/1, the conversion units of angular position and velocity are:
  - APunit (argument of `APx?`, `Gx:`): 1 degree = 19583.33333.. APunits (1 arcsec = 5.44... APunits)
  - speed (`SPDx:`): `SPDx:1` is 0.00079787 deg/s, `SPDx:5000` =~ 4 deg/s
## How to adjust worm-gear
- AstroBaby's guide how to adjust set screws: https://www.astro-baby.com/EQ6%20rebuild%20guide/EQ6%20worm%20alignment.htm
  - the final go/nogo threshold shall be tested at maximum nominal speed, such as `SPDx:5000`

# SW tools
## TrackPV.java
Calls OreKit to calculate position+velocity vectors over specified time period
```
javac -classpath '/opt/orekit/hipparchus-1.8-bin/*:/opt/orekit/orekit-10.3.jar' TrackPV.java
java -classpath '/opt/orekit/hipparchus-1.8-bin/*:/opt/orekit/orekit-10.3.jar:.' TrackPV 2023-05-25T20:00:00 720 > pv_iss_2023-05-25T200000_720.txt
```
TrackPV has been upgraded with a simple trigonometric calculation, allowing to transform the topocentric coordinate vector (XYZ position) to Az/El or Equatorial (HA/DE, meaning Hour Angle and Declination) angular coordinates. By default the Equatorial (HA/DE) is hard-coded.

For convenience, the TrackPV may be called via `calc.sh` allowing simple search for the TLE in `tle.txt` satellite element catalogue. Usage:
```
./calc.sh h-2a 2023-06-04T21:43:00 600 > tracks/h-2a_2023-06-04T214300_600.eq
```
`calc.sh` accepts a satellite id as the first argument. Either a string to be matched, or SATCAT/NORAD ID. So e.g. "H-2A" or "38341" are both valid identifiers.
**Depends on OreKit** and related libs in `/opt/orekit`:
```
hipparchus-1.8-bin
orekit-10.3.jar
orekit-data
```
## pv2joint.jl
Translates XYZ position vector to mount axes (topocentric frame with possibly tilted "a" axis)
```
julia> include("pv2joint.jl")
julia> writedlm("pv_iss_2023-05-25T200000_720.ab", th);
```
Abandoned in favour of `TrackPV.java`.

## lxrmount
Tracking 2-axis trajectory with RoCon & astro mount. Has been modified to use TCP/IP instead of USB to command RoCon. Two versions exist:
- `lxrmount_real` -- tracking in real-time based on local Linux `CLOCK_REALTIME`. (Beware of extrapolation when running far from the interval covered by track file)
- `lxrmount_fake` -- tracking in simulated time, meaning the instant of launching to be coincident with first line of the track file

### Safety and interruption
- `Ctrl-C` (SIGINT) = exit, but stop the motors (`SPDx:0`) before exiting
- `Ctrl-\` (SIGQUIT) = hard quit, when e.g. the connection to RoCon is broken and stopping the motors may not be successful

- `make lxrmount`
- `./lxrmount_real rocon.local tracks/h-2a_2023-06-04T214300_600.eq`

# Authors
- this pile of scripts and programs: Marek Peca
- LX RoCon and unerlying PXMC motion control infrastructure: Pavel Pisa
- first testers and users for W/E-band antenna satellite tracking: Vaclav Valenta and Hugo Deberge
