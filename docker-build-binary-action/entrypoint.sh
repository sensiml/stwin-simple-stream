#!/bin/bash

cmake $GITHUB_WORKSPACE/STWin-SimpleStream
make stwin-sensiml-simplestream-data-collection.elf -j
make stwin-sensiml-simplestream-recognition.elf -j
