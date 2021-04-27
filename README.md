# STWin Simple Streaming Interface

Simple Streaming Firmware for the ST Wireless Industrial Node development board.

## Building

The application is built primarily using the CMake build system.

### Using CMake

From the root directory, you can run the command:

`cmake STWin-SimpleStream/`

If you have a preferred ARM toolchain to use, you can also specify the toolchain file:

`cmake -DCMAKE_TOOLCHAIN_FILE=<PATH_TO_FILE> STWin-SimpleStream/`

The default generator is a Makefile for CMake. After this building the binary works by using Make.

- For data collection:
  - `make stwin-sensiml-simplestream-data-collection.elf -j`
- For recognition:
  - `make stwin-sensiml-simplestream-recognition.elf -j`

## Using Docker

SensiML provides a public docker image using the current version of ARM-GCC utilized by STMicroelectronics CubeIDE. To run this locally, you will need to have Docker installed. You can then use the Dockerfile in `docker-build-binary-action`:

```
cd docker-build-binary-action
docker build . -t my-stwin-builder
cd ..
docker run -v `pwd`:/build -e GITHUB_WORKSPACE=/build my-stwin-builder
```

The binary for both recognition and data collection will be built. They will be located (by default) in `STWin-SimpleStream/docker-build`

## Enabling Audio & IMU

In the file `STWin-SimpleStream/inc/main.h`, enable/disable audio by changing the define: `#define ENABLE_AUDIO 0` to be 1 or 0. If audio is disabled, the firmware will build for the on-board IMU.
