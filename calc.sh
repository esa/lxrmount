#!/bin/bash
# usage: ./calc.sh 25544U 2023-05-25T20:00:00 720

tle_file=tle.txt

if grep -qi ^$1 $tle_file; then
    tle1=$(grep -i -A2 ^$1 $tle_file | tail -2 | head -1)
    tle2=$(grep -i -A2 ^$1 $tle_file | tail -1)
else
    if grep -qi "^1 $1" $tle_file; then
	tle1=$(grep -i -A1 "^1 $1" $tle_file | tail -2 | head -1)
	tle2=$(grep -i -A1 "^1 $1" $tle_file | tail -1)
    else
	echo "sat $1 not found in $tle_file" >&2
	exit 1
    fi
fi

echo "TLE:$tle1" >&2
echo "TLE:$tle2" >&2

java -classpath '/opt/orekit/hipparchus-1.8-bin/*:/opt/orekit/orekit-10.3.jar:.' TrackPV $2 $3 "$tle1" "$tle2"
