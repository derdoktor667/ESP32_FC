# File: Dockerfile
# This Dockerfile adds python, make, and other build essentials to the base image.

FROM archlinux:latest

RUN pacman -Syu --noconfirm && \
    pacman -S --noconfirm git openssh wget python base-devel

ARG SOURCE_REPO_URL
ARG FQBN="esp32:esp32:esp32"
ARG ARDUINO_CORE="esp32:esp32"
ARG ESP32_BOARD_URL="https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"
ARG SKETCH_NAME="ESP32_FC"
ARG ARDUINO_CLI_VERSION="0.35.3"

RUN wget https://github.com/arduino/arduino-cli/releases/download/v${ARDUINO_CLI_VERSION}/arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz -O arduino-cli.tar.gz && \
    tar -xzf arduino-cli.tar.gz && \
    mv arduino-cli /usr/local/bin/ && \
    rm arduino-cli.tar.gz

RUN arduino-cli config init && \
    arduino-cli core update-index --additional-urls ${ESP32_BOARD_URL} && \
    arduino-cli core install ${ARDUINO_CORE} --additional-urls ${ESP32_BOARD_URL}

RUN git clone --recurse-submodules ${SOURCE_REPO_URL} /app/${SKETCH_NAME}

RUN SKETCHBOOK_PATH=$(arduino-cli config dump | grep 'user:' | awk '{print $2}') && \
    mkdir -p ${SKETCHBOOK_PATH}/libraries && \
    cd /app/${SKETCH_NAME} && \
    for SUBMODULE_PATH in $(git config --file .gitmodules --get-regexp path | awk '{ print $2 }'); do \
    cp -r "${SUBMODULE_PATH}" "${SKETCHBOOK_PATH}/libraries/" ; \
    done

WORKDIR /app/${SKETCH_NAME}

RUN arduino-cli compile --fqbn ${FQBN} . --log
