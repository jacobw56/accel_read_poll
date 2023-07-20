#!/bin/bash

RED='\033[0;31m'
GRN='\033[0;32m'
LTRED='\033[1;31m'
LTGRN='\033[1;32m'
YLLW='\033[1;33m'
PRPL='033[0;35m'
LTPRPL='\033[1;35m'
CYAN='\033[0;36m'
LTCYAN='\033[1;36m'
NC='\033[0m'

# Stop on error!
set -e

# change to build
mkdir build
cd build

# Run cmake with defines
cmake \
    -DCMAKE_TOOLCHAIN_FILE="../toolchain.cmake" \
    -DDFU_FILENAME=${DFU_NAME} \
    -DKEY_DIR=${BOOTLOADER_KEY_DIR} \
    -DARM_NONE_EABI_PATH=${ARM_NONE_EABI_PATH} \
    -DSDK_ROOT_DIR=${NRF_SDK_DIR} \
    ..

# make project and show pcts
make PRETTY=1

# Build a DFU zip package
nrfutil pkg generate \
    --application ./bootloader_template.hex \
    --application-version-string "1.0.0" \
    --hw-version 2 \
    --sd-req 0x0100 \
    --sd-id 0x0100 \
    --key-file ${BOOTLOADER_KEY_DIR}/bootloader_private.key \
    ./${DFU_NAME}.zip