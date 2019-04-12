#!/bin/bash

cd tools
make
./simulator.bin ../config/car.txt ../config/road.txt ../config/cross.txt ../config/presetAnswer.txt ../config/answer.txt
