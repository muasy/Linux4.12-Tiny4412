#!/bin/bash

# Developer: SY
# Data     : 2017-7-5 19:30:37
# Function : Auto Make Project

echo "------------------------------"
echo "make"
make ARCH=arm uImage LOADADDR=0X40008000 -j4

echo "make dts"
make ARCH=arm dtbs

echo "backup config"
cp .config tiny4412_defconfig

echo "over"


