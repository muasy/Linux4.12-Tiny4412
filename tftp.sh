#!/bin/bash

# Developer: SY
# Data     : 2017-7-12 20:47:29
# Function : TFTP Upgrade

TFTP_PATH=/opt/tftpboot/

echo "------------------------------"
echo "TFTP uImage"
cp arch/arm/boot/uImage $TFTP_PATH

echo "------------------------------"
echo "TFTP ramdisk"
cp /opt/rootfs/ramdisk.img $TFTP_PATH

echo "------------------------------"
echo "TFTP exynos4412-tiny4412.dtb"
cp arch/arm/boot/dts/exynos4412-tiny4412.dtb $TFTP_PATH

echo "------------------------------"
echo "done!"


