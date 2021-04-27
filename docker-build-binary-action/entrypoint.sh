#!/bin/bash
cmake -E make_directory $GITHUB_WORKSPACE/STWin-SimpleStream/docker-build
cmake $GITHUB_WORKSPACE/STWin-SimpleStream -B $GITHUB_WORKSPACE/STWin-SimpleStream/docker-build
cd $GITHUB_WORKSPACE/STWin-SimpleStream/docker-build
make stwin-sensiml-simplestream-data-collection.elf -j
make stwin-sensiml-simplestream-recognition.elf -j
