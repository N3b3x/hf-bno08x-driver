#!/usr/bin/env bash
set -e
platform=$1

CXX=g++
CC=gcc
case "$platform" in
  esp32)
    CXX=xtensa-esp32-elf-g++
    CC=xtensa-esp32-elf-gcc
    ;;
  stm32)
    CXX=arm-none-eabi-g++
    CC=arm-none-eabi-gcc
    ;;
  arduino)
    CXX=avr-g++
    CC=avr-gcc
    ;;
  *)
    ;;
esac

CXXFLAGS="-I./src -I./src/sh2 -I./src/dfu -I./src/rvc -I./examples/common -std=c++11"
CFLAGS="-I./src -I./src/sh2 -I./src/dfu -I./src/rvc"

mkdir -p build

main_file="examples/common/main.cpp"
if [ -f "examples/$platform/main.cpp" ]; then
  main_file="examples/$platform/main.cpp"
elif [ -f "examples/$platform/main/main.cpp" ]; then
  main_file="examples/$platform/main/main.cpp"
fi

$CXX $CXXFLAGS -c "$main_file" -o build/main.o
$CXX $CXXFLAGS -c src/BNO085.cpp -o build/BNO085.o
$CXX $CXXFLAGS -c src/dfu/dfu_bno.cpp -o build/dfu_bno.o
$CXX $CXXFLAGS -c src/dfu/dfu_fsp200.cpp -o build/dfu_fsp200.o
$CXX $CXXFLAGS -c src/rvc/rvc.cpp -o build/rvc.o
$CC $CFLAGS -c src/dfu/firmware-bno.c -o build/firmware-bno.o
$CC $CFLAGS -c src/dfu/firmware-fsp.c -o build/firmware-fsp.o
$CC $CFLAGS -c src/sh2/euler.c -o build/euler.o
$CC $CFLAGS -c src/sh2/sh2.c -o build/sh2.o
$CC $CFLAGS -c src/sh2/sh2_SensorValue.c -o build/sh2_SensorValue.o
$CC $CFLAGS -c src/sh2/sh2_util.c -o build/sh2_util.o
$CC $CFLAGS -c src/sh2/shtp.c -o build/shtp.o

echo "Compiled for $platform"
