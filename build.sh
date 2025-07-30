#/bin/bash

if [ $(id -u) -ne 0 ]
  then echo Please run this script as root or using sudo!
  exit
fi

make clean
make

sudo dtoverlay -r ads1256
sudo rmmod ads1256

sudo insmod ads1256.ko

dtc -@ -I dts -O dtb -o ads1256.dtbo ads1256-overlay.dts
sudo dtoverlay ./ads1256.dtbo
ls /sys/kernel/config/device-tree/overlays/

gcc -o read_ads1256 read_ads1256.c -lm

ls /dev/ads1256*
ls /dev/spidev*