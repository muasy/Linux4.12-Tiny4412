#!/bin/bash

#################################################
# Developer: SY
# Data     : 2017-7-5 19:30:37
# Function : Auto Make Project
#################################################

echo "###########################################"
echo "make"
# All DEBUG(KCFLAGS=-DDEBUG)
make ARCH=arm uImage LOADADDR=0X40008000 -j4

echo "###########################################"
echo "make dts"
make ARCH=arm dtbs

echo "###########################################"
echo "backup config"
cp .config tiny4412_defconfig

echo "###########################################"
echo "cp file to tftpboot"
./tftp.sh

echo "done!"


