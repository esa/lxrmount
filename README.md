# LX RoCon -- tracking satellites and celestial objects

## Setup
- PiKRON LX RoCon, 2 axes in use, mode=8 (stepper motor without IRC)
- Synta (branded SkyWatcher) NEQ6 mount (kinematic equivalent of HEQ5)

## Conventions
- Ground--Axis "a"--Axis "b"--Sky (a=Az or Hour Angle, b=El or Dec)
- "a" connected as "A", "b" connected as "B"

# Manual commanding of LX RoCon
## Comms I/O
- how to issue a command:
a) echo SOME:COMMAND > /dev/ttyACM0 # beware, ttyACM* suffix may vary in case of e.g. disconnecting while in useb) 
b) terminal program, e.g. minicom -D /dev/ttyACM0 # and then write one or more commands/queries interactively
- when using terminal, character echo is handy; if disabled by previous program, re-enable:
`echo 'ECHO:1' > /dev/ttyACM0`
- case matters, CapsLock is your friend
## Survival
- `echo STOP: > /dev/ttyACM0` stops motion, but motors kept powered, holding position
- `echo RELEASE: > /dev/ttyACM0` disconnect motors, no current, equivalent to disconnected cable
- **do not run axis specific commands before configuring the axes!**
Basic configuration: `./lxr_init.sh > /dev/ttyACM0`
- help # (yes, lowercase) is an online help, you can also ask for help to a specific command, e.g. help AP
## Axis suffices
- most of RoCon cmd/query commands need suffix designating axis: A,B,C,...
-- e.g. query position: command is "AP", to query position of axis B: APB?
-- or: set speed of axis A to 100: SPDB:100

# LX RoCon homepage and docs
- https://www.pikron.com/pages/products/motion_control/lx_rocon.html
- https://gitlab.com/pikron/projects/lx_cpu/lx-rocon
- similar command set to earlier model MARS-8: http://cmp.felk.cvut.cz/~pisa/mars8/mars8_man_cz.html

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
