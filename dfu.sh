#!/bin/bash

# Developer: SY
# Data     : 2017-7-8 12:21:09
# Function : DFU 

echo "------------------------------"
echo "DFU List"
dfu-util -l

echo "------------------------------"
echo "DFU uImage"
dfu-util -D arch/arm/boot/uImage -a 0

echo "------------------------------"
echo "DFU ramdisk"
dfu-util -D /opt/rootfs/ramdisk.img -a 1

echo "------------------------------"
echo "DFU exynos4412-tiny4412.dtb"
dfu-util -D arch/arm/boot/dts/exynos4412-tiny4412.dtb -a 2

echo "------------------------------"
echo "done!"


