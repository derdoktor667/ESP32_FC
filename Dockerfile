# This Dockerfile builds an ESP32 Arduino sketch using arduino-cli.

FROM archlinux:latest AS builder

ARG SOURCE_REPO_URL
ARG FQBN="esp32:esp32:esp32"
ARG ARDUINO_CORE="esp32:esp32"
ARG ESP32_BOARD_URL="https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json"
ARG SKETCH_NAME="ESP32_FC"
ARG ARDUINO_CLI_VERSION="0.35.3"

ENV PATH="/usr/local/bin:${PATH}"
WORKDIR /src

# Install required packages and clean cache in one layer
RUN pacman-key --init && \
    pacman-key --populate archlinux && \
    pacman -Syu --noconfirm && \
    pacman -S --noconfirm --needed git openssh wget python base-devel make unzip && \
    pacman -Scc --noconfirm

# Install arduino-cli with verification
RUN wget -q "https://github.com/arduino/arduino-cli/releases/download/v${ARDUINO_CLI_VERSION}/arduino-cli_${ARDUINO_CLI_VERSION}_Linux_64bit.tar.gz" -O /tmp/arduino-cli.tar.gz && \
    tar -xzf /tmp/arduino-cli.tar.gz -C /tmp && \
    mv /tmp/arduino-cli /usr/local/bin/ && \
    chmod +x /usr/local/bin/arduino-cli && \
    rm -f /tmp/arduino-cli.tar.gz && \
    arduino-cli version

# Verify SOURCE_REPO_URL
RUN if [ -z "${SOURCE_REPO_URL}" ]; then echo "ERROR: SOURCE_REPO_URL not set"; exit 1; fi

# Clone repository with verification
RUN git clone --recurse-submodules --shallow-submodules --depth 1 "${SOURCE_REPO_URL}" "/src/${SKETCH_NAME}" && \
    ls -la "/src/${SKETCH_NAME}"

WORKDIR /src/${SKETCH_NAME}

# Setup Arduino environment
RUN arduino-cli config init && \
    arduino-cli core update-index --additional-urls "${ESP32_BOARD_URL}" && \
    arduino-cli core install "${ARDUINO_CORE}" --additional-urls "${ESP32_BOARD_URL}" && \
    # List available libraries for verification
    arduino-cli lib search ESP32 && \
    # Install required libraries - using correct names
    arduino-cli lib install \
        "WiFiManager" \
        "ESP32Servo" \
        "ESP32AnalogRead" \
        "AsyncTCP-ESP32" && \
    # Verify installations
    arduino-cli core list && \
    arduino-cli lib list

# Setup libraries from submodules
RUN SKETCHBOOK_PATH=$(arduino-cli config dump | awk '/user:/ {print $2}') && \
    mkdir -p "${SKETCHBOOK_PATH}/libraries" && \
    if [ -f .gitmodules ]; then \
    git submodule foreach --recursive 'cp -r \"$toplevel/$path\" \"'"${SKETCHBOOK_PATH}"'/libraries/\" || true'; \
    fi

# Pre-build verification and compilation
RUN mkdir -p /out && \
    echo "=== Build Environment Info ===" && \
    echo "Working Directory: $(pwd)" && \
    echo "Directory Contents:" && ls -la && \
    echo "Arduino CLI Version:" && arduino-cli version && \
    echo "Installed Cores:" && arduino-cli core list && \
    echo "Installed Libraries:" && arduino-cli lib list && \
    echo "=== Starting Compilation ===" && \
    arduino-cli compile \
    --fqbn "${FQBN}" \
    --output-dir /out \
    --verbose \
    --log-level debug \
    --warnings all \
    . || \
    { echo "=== Build Error Details ==="; \
    echo "1. Current Directory: $(pwd)"; \
    echo "2. Sketch Files:"; \
    find . -name "*.ino" -o -name "*.cpp" -o -name "*.h"; \
    echo "3. Arduino CLI Config:"; \
    arduino-cli config dump; \
    echo "4. Build Logs:"; \
    find /out -type f -name "*.log" -exec cat {} \; 2>/dev/null || true; \
    exit 1; \
    }

# Cleanup
RUN rm -rf /var/cache/pacman/pkg/* /tmp/*

# Final stage
FROM alpine:latest AS final
ARG SOURCE_REPO_URL
ARG SKETCH_NAME="ESP32_FC"
WORKDIR /out
COPY --from=builder /out /out
LABEL org.opencontainers.image.source="${SOURCE_REPO_URL}"
CMD ["sh", "-c", "ls -la /out || true"]
