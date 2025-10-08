let port;
let reader;
let writer;
let isConnected = false;
let readLoopActive = false;
let liveDataIntervalId = null;
let readableStreamClosed; // New variable to store the readable stream closed promise
let writableStreamClosed; // New variable to store the writable stream closed promise
const LIVE_DATA_INTERVAL_MS = 100; // Interval for sending 'debug' command

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
const liveDataToggle = document.getElementById('liveDataToggle');

const rollKpInput = document.getElementById('rollKp');
const rollKiInput = document.getElementById('rollKi');
const rollKdInput = document.getElementById('rollKd');
const pitchKpInput = document.getElementById('pitchKp');
const pitchKiInput = document.getElementById('pitchKi');
const pitchKdInput = document.getElementById('pitchKd');
const yawKpInput = document.getElementById('yawKp');
const yawKiInput = document.getElementById('yawKi');
const yawKdInput = document.getElementById('yawKd');

// --- Page Switching ---
function showPage(pageId) {
    if (pageId === 'welcomePage') {
        welcomePage.style.display = 'block';
        configPage.style.display = 'none';
    } else if (pageId === 'configPage') {
        welcomePage.style.display = 'none';
        configPage.style.display = 'block';
    }
}

// --- UI State Management ---
function updateUIConnected(connected) {
    isConnected = connected;
    connectButton.disabled = connected; // Only on welcome page
    disconnectButton.disabled = !connected;
    cliCommandInput.disabled = !connected;
    sendCommandButton.disabled = !connected;
    setPIDsButton.disabled = !connected;
    liveDataToggle.disabled = !connected; // Enable/disable live data toggle
    rollKpInput.disabled = !connected;
    rollKiInput.disabled = !connected;
    rollKdInput.disabled = !connected;
    pitchKpInput.disabled = !connected;
    pitchKiInput.disabled = !connected;
    pitchKdInput.disabled = !connected;
    yawKpInput.disabled = !connected;
    yawKiInput.disabled = !connected;
    yawKdInput.disabled = !connected;

    // Update status text on both pages
    connectionStatus.textContent = connected ? 'Connected' : 'Disconnected';
    configPageConnectionStatus.textContent = connected ? 'Connected' : 'Disconnected';

    if (!connected) {
        // If disconnected, ensure live data is off and show welcome page
        liveDataToggle.checked = false;
        toggleLiveData(false); // Explicitly turn off live data
        showPage('welcomePage');
    } else {
        showPage('configPage');
    }
}

// --- Live Data Control ---
function toggleLiveData(enable) {
    if (enable) {
        writeToSerial('log on'); // Enable global logging
        writeToSerial(`set log.interval ${LIVE_DATA_INTERVAL_MS}`); // Set interval for continuous logging
        liveDataIntervalId = setInterval(() => {
            writeToSerial('debug'); // Request debug data periodically
        }, LIVE_DATA_INTERVAL_MS);
    } else {
        clearInterval(liveDataIntervalId);
        liveDataIntervalId = null;
        writeToSerial('log off'); // Disable global logging
        writeToSerial('set log.interval 0'); // Ensure continuous logging is off
    }
}

// --- Serial Communication Functions ---
async function connectSerial() {
    connectionStatus.textContent = 'Connecting...'; // Update status on welcome page
    try {
        port = await navigator.serial.requestPort();        // Add event listeners for connection loss
        port.onconnectionlost = (event) => {
            console.error('Serial port connection lost:', event);
            appendSerialOutput('Serial port connection lost unexpectedly.\n');
            updateUIConnected(false);
        };
        port.onclose = (event) => {
            console.log('Serial port closed.', event);
            appendSerialOutput('Serial port closed.\n');
            updateUIConnected(false);
        };

        await port.open({ baudRate: 115200 });

        // Wait for 2 seconds to allow ESP32 to reboot and stabilize
        await new Promise(resolve => setTimeout(resolve, 2000));

        const textDecoder = new TextDecoderStream();
        readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
        reader = textDecoder.readable.getReader();

        const textEncoder = new TextEncoderStream();
        writableStreamClosed = textEncoder.readable.pipeTo(port.writable);
        writer = textEncoder.writable.getWriter();
        updateUIConnected(true);
        appendSerialOutput('Connected to serial port.\n');
        readFromSerial();
    } catch (error) {
        console.error('Error connecting to serial port:', error);
        appendSerialOutput(`Error: ${error.message}\n`);
        updateUIConnected(false);
    }
}

let readLoopPromise = null; // To hold the promise of the read loop

// ... (other code) ...

async function disconnectSerial() {
    try {
        if (reader) {
            readLoopActive = false; // Signal the read loop to stop
            await reader.cancel(); // Cancel any pending read operation
            if (readLoopPromise) {
                await readLoopPromise; // Wait for the read loop to fully terminate
            }
            reader.releaseLock();
            reader = null;
        }
        if (writer) {
            await writer.close();
            writer.releaseLock();
            writer = null;
        }
        // Await the pipeTo promises to ensure streams are fully closed
        if (readableStreamClosed) {
            await readableStreamClosed.catch(() => { }); // Catch errors if pipe already broken
        }
        if (writableStreamClosed) {
            await writableStreamClosed.catch(() => { }); // Catch errors if pipe already broken
        }

        if (port) {
            await port.close();
            port = null;
        }
        updateUIConnected(false);
        appendSerialOutput('Disconnected from serial port.\n');
    } catch (error) {
        console.error('Error disconnecting from serial port:', error);
        appendSerialOutput(`Error: ${error.message}\n`);
    }
}

async function readFromSerial() {
    readLoopActive = true;
    console.log('Starting read loop.');
    readLoopPromise = new Promise(async (resolve) => {
        while (port && readLoopActive) {
            try {
                const { value, done } = await reader.read();
                if (done) {
                    console.log('Read loop terminated: done is true.');
                    break;
                }
                appendSerialOutput(value);
            } catch (error) {
                if (error.name === 'NetworkError' && error.message.includes('The port is already closed')) {
                    console.log('Serial port closed during read operation (NetworkError).');
                } else if (error.name === 'AbortError') {
                    console.log('Read operation aborted (AbortError).');
                } else {
                    console.error('Error reading from serial port:', error);
                    appendSerialOutput(`Read Error: ${error.message}\n`);
                }
                break;
            }
        }
        readLoopActive = false;
        console.log('Read loop finished.');
        resolve();
    });
    return readLoopPromise;
}
async function writeToSerial(data) {
    if (writer && isConnected) {
        try {
            await writer.write(data + '\n');
            // Only log commands that are not 'debug' or 'set log.interval' to reduce verbosity
            if (!data.startsWith('debug') && !data.startsWith('set log.interval')) {
                appendSerialOutput(`Sent: ${data}\n`);
            }
        } catch (error) {
            console.error('Error writing to serial port:', error);
            appendSerialOutput(`Write Error: ${error.message}\n`);
        }
    } else {
        appendSerialOutput('Not connected to serial port.\n');
    }
}

function appendSerialOutput(message) {
    serialOutput.textContent += message;
    serialOutput.scrollTop = serialOutput.scrollHeight; // Auto-scroll to bottom
}

// --- Event Listeners ---
connectButton.addEventListener('click', connectSerial);
disconnectButton.addEventListener('click', disconnectSerial);

liveDataToggle.addEventListener('change', () => {
    toggleLiveData(liveDataToggle.checked);
});

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
    writeToSerial(`set pid.roll.kp ${rollKpInput.value}`);
    writeToSerial(`set pid.roll.ki ${rollKiInput.value}`);
    writeToSerial(`set pid.roll.kd ${rollKdInput.value}`);
    writeToSerial(`set pid.pitch.kp ${pitchKpInput.value}`);
    writeToSerial(`set pid.pitch.ki ${pitchKiInput.value}`);
    writeToSerial(`set pid.pitch.kd ${pitchKdInput.value}`);
    writeToSerial(`set pid.yaw.kp ${yawKpInput.value}`);
    writeToSerial(`set pid.yaw.ki ${yawKiInput.value}`);
    writeToSerial(`set pid.yaw.kd ${yawKdInput.value}`);
    writeToSerial('save'); // Save settings after updating PIDs
});

// Initial UI state
updateUIConnected(false);
showPage('welcomePage'); // Show welcome page initially