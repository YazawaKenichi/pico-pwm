#!/bin/bash

cd /home/yazawa/pico/pico-pwm
rm -rf ./build
mkdir build
cd build
cmake ..
make -j4
PICO_DIR="/media/yazawa/RPI-RP2"

while [ ! -d "$PICO_DIR" ] ; do
    echo -e "Pico が書き込みモードで接続されていません\r"
    sleep 1
done

echo "Done!"

cp *.uf2 /media/yazawa/RPI-RP2

