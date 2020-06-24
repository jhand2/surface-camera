#!/bin/bash

RES="2592x1944"
FMT="SGRBG10"

MDEV=/dev/media0
SDEV=$(sudo media-ctl -d $MDEV -e "ov5693 19-0036")

ENTITIES=$(sudo media-ctl -d /dev/media0 -p | grep "entity")

OV5693=$(echo "$ENTITIES" | \
         grep "ov5693" | \
         awk '{print substr($3, 1, length($3)-1)}')

CSI2=$(echo "$ENTITIES" | \
         grep "csi2 0" | \
         awk '{print substr($3, 1, length($3)-1)}')

sudo media-ctl -d $MDEV -l "$OV5693:0 -> $CSI2:0[1]"
sudo media-ctl -d $MDEV -V "$OV5693:0 [fmt:$FMT/$RES]"
sudo media-ctl -d $MDEV -V "$CSI2:0 [fmt:$FMT/$RES]"
sudo media-ctl -d $MDEV -V "$CSI2:1 [fmt:$FMT/$RES]"

#sudo yavta -data-prefix -u -c10 -n1 -I -s $RES \
    #--file=/tmp/frame-#.bin -f IPU3_$FMT /dev/video0

# Uncomment for debugging purposes
sudo strace yavta -data-prefix -u -c10 -n1 -I -s $RES \
    --file=/tmp/frame-#.bin -f IPU3_$FMT /dev/video0
