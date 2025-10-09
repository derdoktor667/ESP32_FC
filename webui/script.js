let port;
let reader;
let writer;
let isConnected = false;
let settingsPromiseResolver = null;

// Page elements
const welcomePage = document.getElementById('welcomePage');
const configPage = document.getElementById('configPage');

// Welcome Page elements
const connectButton = document.getElementById('connectButton');
const connectionStatus = document.getElementById('connectionStatus');

// Config Page elements
const disconnectButton = document.getElementById('disconnectButton');
const configPageConnectionStatus = document.getElementById('configPageConnectionStatus');
const serialOutput = document.getElementById('serialOutput');
const cliCommandInput = document.getElementById('cliCommandInput');
const sendCommandButton = document.getElementById('sendCommandButton');
const setPIDsButton = document.getElementById('setPIDsButton');
const dshotModeSelect = document.getElementById('dshotModeSelect');
const setMotorSettingsButton = document.getElementById('setMotorSettingsButton');

// PID Inputs
const pidInputs = {
    roll: { kp: document.getElementById('rollKp'), ki: document.getElementById('rollKi'), kd: document.getElementById('rollKd') },
    pitch: { kp: document.getElementById('pitchKp'), ki: document.getElementById('pitchKi'), kd: document.getElementById('pitchKd') },
    yaw: { kp: document.getElementById('yawKp'), ki: document.getElementById('yawKi'), kd: document.getElementById('yawKd') }
};

// Live Data Spans
const liveRoll = document.getElementById('live-roll');
const livePitch = document.getElementById('live-pitch');
const liveYaw = document.getElementById('live-yaw');
const liveSetpointRoll = document.getElementById('live-setpoint-roll');
const liveSetpointPitch = document.getElementById('live-setpoint-pitch');
const liveSetpointYaw = document.getElementById('live-setpoint-yaw');
const liveSetpointThrottle = document.getElementById('live-setpoint-throttle');
const liveArmed = document.getElementById('live-armed');
const liveFailsafe = document.getElementById('live-failsafe');
const liveMode = document.getElementById('live-mode');
const liveLoopTime = document.getElementById('live-loop-time');
const liveImuTemp = document.getElementById('live-imu-temp');
const liveVoltage = document.getElementById('live-voltage');
const liveCurrent = document.getElementById('live-current');
const liveRssi = document.getElementById('live-rssi');
const liveArmedTime = document.getElementById('live-armed-time');
const liveCpuLoad = document.getElementById('live-cpu-load');
const liveMotorOutput = [
    document.getElementById('live-motor-0'),
    document.getElementById('live-motor-1'),
    document.getElementById('live-motor-2'),
    document.getElementById('live-motor-3')
];
const liveWarnings = document.getElementById('live-warnings');
const liveErrors = document.getElementById('live-errors');
const receiverChannelsDiv = document.getElementById('receiver-channels');

// --- Page Switching ---
function showPage(pageId) {
    welcomePage.style.display = (pageId === 'welcomePage') ? 'block' : 'none';
    configPage.style.display = (pageId === 'configPage') ? 'block' : 'none';
}

// --- UI State Management ---
function updateUIConnected(connected) {
    isConnected = connected;
    connectButton.disabled = connected;
    disconnectButton.disabled = !connected;
    cliCommandInput.disabled = !connected;
    sendCommandButton.disabled = !connected;
    setPIDsButton.disabled = !connected;
    dshotModeSelect.disabled = !connected;
    setMotorSettingsButton.disabled = !connected;

    Object.values(pidInputs).forEach(axis => {
        Object.values(axis).forEach(input => input.disabled = !connected);
    });

    connectionStatus.textContent = connected ? 'Connected' : 'Disconnected';
    configPageConnectionStatus.textContent = connected ? 'Connected' : 'Disconnected';

    if (!connected) {
        toggleLiveData(false); // Ensure live data stream is stopped
        showPage('welcomePage');
    } else {
        showPage('configPage');
    }
}

// --- Live Data Control ---
function toggleLiveData(enable) {
    if (enable) {
        writeToSerial('debug on');
    } else {
        writeToSerial('debug off');
    }
}

// --- Serial Communication Functions ---
async function connectSerial() {
    connectionStatus.textContent = 'Connecting...';
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });

        // CRITICAL: Wait for ESP32 to reboot and stabilize after connection
        appendSerialOutput('Port opened. Waiting for device to stabilize...\n');
        await new Promise(resolve => setTimeout(resolve, 2000));

        const textDecoder = new TextDecoderStream();
        port.readable.pipeTo(textDecoder.writable);
        reader = textDecoder.readable.getReader();

        const textEncoder = new TextEncoderStream();
        textEncoder.readable.pipeTo(port.writable);
        writer = textEncoder.writable.getWriter();

        updateUIConnected(true);
        appendSerialOutput('Device connected. Initializing API mode...\n');

        readFromSerial();

        // Enter API mode, get settings, then start live data
        await writeToSerial('api');
        await new Promise(resolve => setTimeout(resolve, 200)); // Increased delay
        await writeToSerial('dumpjson');

        const settingsPromise = new Promise((resolve, reject) => {
            settingsPromiseResolver = { resolve, reject };
        });

        // Wait for settings to be received, with a timeout
        await Promise.race([
            settingsPromise,
            new Promise((_, reject) => setTimeout(() => reject(new Error('Timeout: Did not receive settings from device.')), 3000))
        ]);

        appendSerialOutput('Settings received, starting live data...\n');
        toggleLiveData(true);

    } catch (error) {
        console.error('Connection process failed:', error);
        appendSerialOutput(`Error: ${error.message}\n`);
        if (port && port.readable) {
            disconnectSerial();
        }
        updateUIConnected(false);
    }
}

async function disconnectSerial() {
    if (isConnected) {
        toggleLiveData(false); // Stop the live data stream
    }
    if (reader) {
        try { await reader.cancel(); } catch (e) { /* Ignore */ }
        reader = null;
    }
    if (writer) {
        try { await writer.close(); } catch (e) { /* Ignore */ }
        writer = null;
    }
    if (port) {
        try { await port.close(); } catch (e) { /* Ignore */ }
        port = null;
    }
    updateUIConnected(false);
    appendSerialOutput('Disconnected from serial port.\n');
}

async function readFromSerial() {
    let lineBuffer = '';
    while (port && port.readable) {
        try {
            const { value, done } = await reader.read();
            if (done) {
                break;
            }
            lineBuffer += value;
            let lines = lineBuffer.split('\n');
            lineBuffer = lines.pop(); // Keep the last, possibly incomplete line

            for (const line of lines) {
                processLine(line.trim());
            }
        } catch (error) {
            console.error('Error reading from serial port:', error);
            appendSerialOutput(`Read Error: ${error.message}\n`);
            // Do not break here, try to continue reading if possible, or handle specific errors.
            // For now, we'll let the browser's serial API handle the actual port closure.
        }
    }
}

function processLine(line) {
    if (!line) return;

    try {
        const data = JSON.parse(line);
        if (data.settings) {
            populateSettings(data.settings);
            if (settingsPromiseResolver) {
                settingsPromiseResolver.resolve();
                settingsPromiseResolver = null;
            }
        } else if (data.attitude) {
            updateLiveData(data);
        } else if (data.status) { // Handle status messages that are not part of live data stream
            appendSerialOutput(`Status: ${JSON.stringify(data)}\n`);
        } else if (data.error) { // Handle error messages from ESP32
            appendSerialOutput(`ESP32 Error: ${data.error}\n`);
        } else {
            appendSerialOutput(line + '\n');
        }
    } catch (e) {
        console.warn('Non-JSON serial data or malformed JSON:', line, e);
        appendSerialOutput(line + '\n');
    }
}

function populateSettings(settings) {
    if (!settings) return;

    if (settings.pid) {
        for (const axis of ['roll', 'pitch', 'yaw']) {
            if (settings.pid[axis]) {
                pidInputs[axis].kp.value = settings.pid[axis].kp.toFixed(3);
                pidInputs[axis].ki.value = settings.pid[axis].ki.toFixed(3);
                pidInputs[axis].kd.value = settings.pid[axis].kd.toFixed(3);
            }
        }
    }

    if (settings.motor && settings.motor.dshot_mode) {
        dshotModeSelect.value = settings.motor.dshot_mode;
    }
}

function updateLiveData(data) {
    if (!data) return;

    if (data.attitude) {
        liveRoll.textContent = data.attitude.roll.toFixed(2);
        livePitch.textContent = data.attitude.pitch.toFixed(2);
        liveYaw.textContent = data.attitude.yaw.toFixed(2);
    }

    if (data.setpoints) {
        liveSetpointRoll.textContent = data.setpoints.roll.toFixed(2);
        liveSetpointPitch.textContent = data.setpoints.pitch.toFixed(2);
        liveSetpointYaw.textContent = data.setpoints.yaw.toFixed(2);
        liveSetpointThrottle.textContent = data.setpoints.throttle.toFixed(2);
    }

    if (data.status) {
        liveArmed.textContent = data.status.armed ? 'Yes' : 'No';
        liveFailsafe.textContent = data.status.failsafe ? 'Active' : 'No';
        liveMode.textContent = data.status.mode;
    }

    if (data.receiver) {
        receiverChannelsDiv.innerHTML = '';
        data.receiver.forEach((channel, index) => {
            const channelEl = document.createElement('span');
            channelEl.className = 'badge bg-secondary me-2 mb-2';
            channelEl.textContent = `CH${index + 1}: ${channel}`;
            receiverChannelsDiv.appendChild(channelEl);
        });
    }

    if (data.loop_time_us) {
        liveLoopTime.textContent = data.loop_time_us;
    }

    if (data.motor_output) {
        for (let i = 0; i < data.motor_output.length; i++) {
            if (liveMotorOutput[i]) {
                liveMotorOutput[i].textContent = data.motor_output[i];
            }
        }
    }

    if (data.imu_temp) {
        liveImuTemp.textContent = data.imu_temp.toFixed(2);
    }

    if (data.voltage) {
        liveVoltage.textContent = data.voltage.toFixed(2);
    }

    if (data.current) {
        liveCurrent.textContent = data.current.toFixed(2);
    }

    if (data.rssi) {
        liveRssi.textContent = data.rssi;
    }

    if (data.armed_time_s) {
        liveArmedTime.textContent = data.armed_time_s;
    }

    if (data.cpu_load) {
        liveCpuLoad.textContent = data.cpu_load.toFixed(2);
    }

    if (data.warnings && data.warnings.length > 0) {
        liveWarnings.textContent = data.warnings.join(', ');
    } else {
        liveWarnings.textContent = 'None';
    }

    if (data.errors && data.errors.length > 0) {
        liveErrors.textContent = data.errors.join(', ');
    } else {
        liveErrors.textContent = 'None';
    }
}

async function writeToSerial(data) {
    if (writer && isConnected) {
        try {
            await writer.write(data + '\n');
        } catch (error) {
            console.error('Error writing to serial port:', error);
            appendSerialOutput(`Write Error: ${error.message}\n`);
        }
    }
}

function appendSerialOutput(message) {
    serialOutput.textContent += message;
    serialOutput.scrollTop = serialOutput.scrollHeight;
}

// --- Event Listeners ---
connectButton.addEventListener('click', connectSerial);

disconnectButton.addEventListener('click', disconnectSerial);

sendCommandButton.addEventListener('click', () => {
    const command = cliCommandInput.value.trim();
    if (command) {
        writeToSerial(command);
        cliCommandInput.value = '';
    }
});

cliCommandInput.addEventListener('keypress', (e) => {
    if (e.key === 'Enter') {
        sendCommandButton.click();
    }
});

setPIDsButton.addEventListener('click', () => {
    for (const axis of ['roll', 'pitch', 'yaw']) {
        writeToSerial(`set pid.${axis}.kp ${pidInputs[axis].kp.value}`);
        writeToSerial(`set pid.${axis}.ki ${pidInputs[axis].ki.value}`);
        writeToSerial(`set pid.${axis}.kd ${pidInputs[axis].kd.value}`);
    }
    setTimeout(() => writeToSerial('save'), 100);
});

setMotorSettingsButton.addEventListener('click', () => {
    const selectedDShotMode = dshotModeSelect.value;
    writeToSerial(`set motor.dshot_mode ${selectedDShotMode}`);
    setTimeout(() => writeToSerial('save'), 100);
});

// Initial UI state
updateUIConnected(false);
showPage('welcomePage');