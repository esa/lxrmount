main: lxrmount_real lxrmount_fake

all: main stellio TrackPV.class

lxrmount_real: lxrmount.c
	gcc -ggdb -Wall -Wno-unused-variable -O -o $@ $< -lm -lpthread

lxrmount_fake: lxrmount.c
	gcc -ggdb -Wall -Wno-unused-variable -O -DFAKETIME -o $@ $< -lm -lpthread

stellio: stellio.c
	gcc -Wall -O -o $@ $< -lm -lpthread -lnova

TrackPV.class: TrackPV.java
	javac -classpath '/opt/orekit/hipparchus-1.8-bin/*:/opt/orekit/orekit-10.3.jar' TrackPV.java

clean:
	rm -rf lxrmount_real lxrmount_fake stellio TrackPV.class
