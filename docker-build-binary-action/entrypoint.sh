#!/bin/bash

cmake $GITHUB_WORKSPACE/STWin-SimpleStream -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi-gcc.cmake
make stwin-sensiml-simplestream-data-collection.elf -j
make stwin-sensiml-simplestream-recognition.elf -j
