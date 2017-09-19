#!/bin/bash

#Mysteryagr
#Compile kernel with a build script to make things simple

#mkdir -p out

#Change toolchain path before using build script!
#export CROSS_COMPILE=~/toolchains/arm-eabi-linaro-4.7.3/bin/arm-eabi-

#Enable when needed:
export USE_CCACHE=1

export ARCH=arm ARCH_MTK_PLATFORM=mt6580

#Enable only when needed:
#make clean
#make mrproper
#Or simply delete out directory to clean source

#Defconfig for Blu G
#make -C $PWD O=$PWD/out ARCH=arm d5028m_blu_gmo_defconfig

#Defconfig for Infinix Hot 3
#make -C $PWD O=$PWD/out ARCH=arm x554_defconfig
make ARCH=arm x554_defconfig

#Edit the number according to the number of CPUs you have in your PC:
#make -j4 -C $PWD O=$PWD/out ARCH=arm
make -j4 ARCH=arm
