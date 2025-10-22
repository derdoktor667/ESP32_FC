/**
 * Expected JSON structures from ESP32 Flight Controller API:
 *
 * 1. API Mode Activation:
 *    {"status":"api_mode_activated"}
 *
 * 2. Get Setting Response:
 *    {"get":{"<parameter_name>":<value>}}
 *    Example: {"get":{"pid.roll.kp":800}}
 *    Example: {"get":{"rx.protocol":"IBUS"}}
 *
 * 3. Set Setting Response:
 *    {"set":{"<parameter_name>":<value>,"status":"success|error", "message":"<error_description>"}}
 *    Example: {"set":{"pid.roll.kp":850,"status":"success"}}
 *    Example: {"set":{"rx.protocol":"UNKNOWN","status":"error","message":"Invalid value. Expected: IBUS, PPM"}}
 *
 * 4. All Settings Dump (from 'get_settings' command):
 *    {"settings":{
 *        "pid.roll.kp":800,
 *        "pid.roll.ki":1,
 *        // ... other settings ...
 *        "rx.map.throttle":1,
 *        // ... other channel mappings ...
 *    }}
 *
 * 5. Firmware Version:
 *    {"version":"0.2.5"}
 *
 * 6. System Status (from 'status' command):
 *    {"status":{
 *        "Loop Time (us)":1234,
 *        "CPU Load (%)":5.67,
 *        "Battery Voltage (V)":12.34,
 *        "Current Draw (A)":1.23,
 *        "Free Heap (bytes)":123456,
 *        "Min Free Heap (bytes)":65432
 *    }}
 *
 * 7. Live Data Stream (when in API mode and logging enabled):
 *    {"live_data":{
 *        "attitude":{"roll":X.XX,"pitch":Y.YY,"yaw":Z.ZZ},
 *        "status":{"armed":true|false,"failsafe":true|false,"mode":"ACRO|ANGLE|UNKNOWN"},
 *        "motor_output":[M1,M2,M3,M4],
 *        "receiver_channels":[CH0,CH1,CH2,...]
 *    }}
 *
 * 8. Error Response:
 *    {"error":"<error_message>"}
 */


const log = document.getElementById('log');
const toggleConnectButton = document.getElementById('toggleConnectButton');
const calibrateImuButton = document.getElementById('calibrateImuButton');
const saveSettingsButton = document.getElementById('saveSettingsButton');

let port;
let writer;
let reader;
let isConnected = false;
let receivedSettings = {}; // Global object to store received settings

// Receiver channel names for display
const receiverChannelNames = [
    "Roll", "Pitch", "Throttle", "Yaw", "Arm", "Failsafe", "Flight Mode",
    "Aux 1", "Aux 2", "Aux 3", "Aux 4", "Aux 5", "Aux 6", "Aux 7", "Aux 8", "Aux 9"
];

const expectedSettingKeys = [
    "pid.roll.kp", "pid.roll.ki", "pid.roll.kd",
    "pid.pitch.kp", "pid.pitch.ki", "pid.pitch.kd",
    "pid.yaw.kp", "pid.yaw.ki", "pid.yaw.kd",
    "pid.integral_limit",
    "rates.angle", "rates.yaw", "rates.acro",
    "filter.comp_tau", "gyro.lpf_cutoff_freq", "accel.lpf_cutoff_freq",
    "gyro.lpf_stages", "accel.lpf_stages", "filter.sample_freq",
    "gyro.notch.enable", "gyro.notch.freq", "gyro.notch.q",
    "rx.min", "rx.max", "rx.arming_threshold", "rx.failsafe_threshold",
    "rx.protocol",
    "imu.protocol", "imu.lpf", "imu.rotation",
    "motor.idle_speed", "motor.dshot_mode",
    "enforce_loop_time",
    "cal.mpu_readings", "cal.accel_z_g",
    "log.print_interval", "log.enable", "log.test_string", "bench.run.en"
];

// --- 3D View ---
let scene, camera, renderer, quadcopter;
let targetQuaternion = new THREE.Quaternion();

function createQuadcopterModel() {
    const model = new THREE.Group();

    // Central Body
    const bodyGeometry = new THREE.BoxGeometry(2, 0.5, 3); // Slightly longer body
    const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0x555555, roughness: 0.7 });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    model.add(body);

    // Flight Direction Marker
    const markerGeometry = new THREE.ConeGeometry(0.3, 0.5, 32);
    const markerMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });
    const marker = new THREE.Mesh(markerGeometry, markerMaterial);
    marker.position.set(0, 0.25, -1.75); // Position on the front of the central body
    marker.rotation.x = Math.PI / 2; // Point it forward
    model.add(marker);

    // Function to create a single propeller (Cylinder)
    const createPropeller = (color) => {
        const propGeometry = new THREE.CylinderGeometry(0.7, 0.7, 0.1, 32);
        const propMaterial = new THREE.MeshStandardMaterial({ color: color });
        const propeller = new THREE.Mesh(propGeometry, propMaterial);
        return propeller;
    };

    // Function to create a motor arm assembly
    const createMotorArm = (position, propColor) => {
        const armLength = position.length();

        // Arm
        const armGeometry = new THREE.BoxGeometry(0.2, 0.2, armLength);
        const armMaterial = new THREE.MeshStandardMaterial({ color: 0x777777 });
        const arm = new THREE.Mesh(armGeometry, armMaterial);

        // Position the arm halfway along the vector to the motor and point it outwards
        arm.position.copy(position).multiplyScalar(0.5);
        arm.lookAt(position);
        model.add(arm);

        // Propeller
        const propeller = createPropeller(propColor);
        propeller.position.copy(position);
        propeller.position.y = 0.35; // Slightly above the body plane
        model.add(propeller);
    };

    // Define motor positions
    const motorPositions = [
        { x: 1.5, y: 0, z: -1.5 }, // Front-Right
        { x: -1.5, y: 0, z: -1.5 }, // Front-Left
        { x: -1.5, y: 0, z: 1.5 },  // Rear-Left
        { x: 1.5, y: 0, z: 1.5 }   // Rear-Right
    ];

    // Create arms and propellers, all propellers are grey
    motorPositions.forEach(pos => createMotorArm(new THREE.Vector3(pos.x, pos.y, pos.z), 0xaaaaaa));

    // The model's default orientation (0,0,0) has it facing away from the camera.
    // No initial rotation is needed.

    return model;
}


function init3D() {
    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);

    // Camera
    camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000); // Initial aspect ratio
    camera.position.set(0, 5, 10); // Elevated and further back view
    camera.lookAt(0, 0, 0);

    // Lighting
    const ambientLight = new THREE.AmbientLight(0x606060);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(1, 1, 1);
    scene.add(directionalLight);

    // Quadcopter Model
    quadcopter = createQuadcopterModel();
    scene.add(quadcopter);

    animate();
}

function animate() {
    requestAnimationFrame(animate);

    // Smoothly interpolate the quadcopter's rotation towards the target
    if (quadcopter && !quadcopter.quaternion.equals(targetQuaternion)) {
        quadcopter.quaternion.slerp(targetQuaternion, 0.2); // Adjusted factor for balance
    }

    if (renderer) {
        renderer.render(scene, camera);
    }
}


// --- Tab Handling ---
function openTab(evt, tabName) {
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("tablinks");
    for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
    }
    document.getElementById(tabName).style.display = "block";
    if (evt) {
        evt.currentTarget.className += " active";
    }

    // Lazy init for 3D view
    if (tabName === '3dViewTab' && !renderer) {
        const container = document.getElementById('3d-container');
        container.style.height = '400px'; // Force height

        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        container.appendChild(renderer.domElement);

        camera.lookAt(scene.position);

        const resizeObserver = new ResizeObserver(() => {
            if (!renderer) return;
            const width = container.clientWidth;
            const height = container.clientHeight;
            console.log(`Resizing 3D container to ${width}x${height}`);
            camera.aspect = width / height;
            camera.updateProjectionMatrix();
            renderer.setSize(width, height);
        });
        resizeObserver.observe(container);
    }
}

// Show the Info tab by default
document.addEventListener('DOMContentLoaded', () => {
    // Find the button that opens the Info tab and click it
    const infoButton = Array.from(document.querySelectorAll('.tablinks')).find(btn => btn.textContent === 'Info');
    if (infoButton) {
        infoButton.click();
    }
    init3D();
    initializeSettingEventListeners(); // Add this call

    if (calibrateImuButton) {
        calibrateImuButton.addEventListener('click', async () => {
            if (isConnected && writer) {
                log.textContent += 'Sending MSP_FC_CALIBRATE_IMU command.\n';
                await sendMspMessage(MSP_FC_CALIBRATE_IMU);
                // Reset 3D model orientation
                if (quadcopter) {
                    targetQuaternion.set(0, 0, 0, 1); // Reset target to no rotation
                    quadcopter.quaternion.set(0, 0, 0, 1); // Snap model to no rotation
                }
            } else {
                log.textContent += 'Not connected to device. Cannot calibrate IMU.\n';
            }
        });
    }

    if (saveSettingsButton) {
        saveSettingsButton.addEventListener('click', async () => {
            if (isConnected && writer) {
                log.textContent += 'Sending MSP_FC_SAVE_SETTINGS command.\n';
                await sendMspMessage(MSP_FC_SAVE_SETTINGS);
                location.reload(); // Reload page after saving settings
            } else {
                log.textContent += 'Not connected to device. Cannot save settings.\n';
            }
        });
    }
});

function enableTabs() {
    const tablinks = document.getElementsByClassName('tablinks');
    for (let i = 0; i < tablinks.length; i++) {
        tablinks[i].removeAttribute('disabled');
    }
}


// --- Serial Communication ---
toggleConnectButton.addEventListener('click', async () => {
    if (!isConnected) {
        // Connect logic
        try {
            port = await navigator.serial.requestPort();
            await port.open({ baudRate: 115200 });

            writer = port.writable.getWriter();
            reader = port.readable.getReader();

            toggleConnectButton.textContent = 'Disconnect';
            isConnected = true;
            saveSettingsButton.disabled = false; // Enable save button

            log.textContent += 'Connected to device.\n';

            // Wait for the device to be ready (optional, but good practice)
            await new Promise(resolve => setTimeout(resolve, 100));

            // Get version
            await sendMspMessage(MSP_FC_GET_VERSION);
            // Get status
            await sendMspMessage(MSP_FC_GET_STATUS);
            // Get settings
            await sendMspMessage(MSP_FC_GET_SETTING, new TextEncoder().encode("all")); // Request all settings

            // Start reading from the port
            readLoop();

        } catch (error) {
            log.textContent += `Error: ${error.message}\n`;
            toggleConnectButton.textContent = 'Connect';
            isConnected = false;
            saveSettingsButton.disabled = true; // Disable save button on error
        }
    } else {
        // Disconnect logic
        if (port) {
            try {
                if (reader) {
                    await reader.cancel();
                    reader.releaseLock();
                }
                if (writer) {
                    await writer.close();
                    writer.releaseLock();
                }
                await port.close();

                toggleConnectButton.textContent = 'Connect';
                isConnected = false;
                saveSettingsButton.disabled = true; // Disable save button on disconnect

                log.textContent += 'Disconnected from device.\n';
                // location.reload(); // Reload page after disconnecting
            } catch (error) {
                log.textContent += `Error: ${error.message}\n`;
            }
        }
    }
});

const MSP_IDLE = 0;
const MSP_HEADER_START = 1;
const MSP_HEADER_M = 2;
const MSP_HEADER_X = 3; // For MSPv2
const MSP_FLAGS = 4; // MSPv2 flags byte
const MSP_SIZE = 5; // payload size low byte
const MSP_SIZE_HIGH = 6; // MSPv2 size high byte
const MSP_COMMAND = 7; // command low byte
const MSP_COMMAND_HIGH = 8; // command high byte (MSPv2)
const MSP_PAYLOAD = 9;
const MSP_CHECKSUM = 10; // MSPv1 checksum
const MSP_CRC = 11; // MSPv2 CRC

let mspState = MSP_IDLE;
let mspBytesRead = 0;
let mspExpectedSize = 0;
let mspCommand = 0;
let mspPayload = [];
let mspChecksum = 0; // For MSPv1
let mspCrc = 0;      // For MSPv2
let isMspV2 = false; // Flag to indicate if current message is MSPv2

function crc8_dvb_s2(crc, byte) {
    crc ^= byte;
    for (let i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5; // DVB-S2 polynomial
        } else {
            crc <<= 1;
        }
    }
    return crc & 0xFF;
}

async function readLoop() {
    try {
        while (true) {
            const { value, done } = await reader.read();
            if (done) break;

            for (let i = 0; i < value.length; i++) {
                const byte = value[i];
                console.log(`Received byte: 0x${byte.toString(16)}, mspState: ${mspState}`);

                switch (mspState) {
                    case MSP_IDLE:
                        if (byte === '$'.charCodeAt(0)) {
                            mspState = MSP_HEADER_START;
                        }
                        break;
                    case MSP_HEADER_START:
                        if (byte === 'M'.charCodeAt(0)) {
                            isMspV2 = false;
                            mspState = MSP_HEADER_M;
                        } else if (byte === 'X'.charCodeAt(0)) {
                            isMspV2 = true;
                            mspState = MSP_HEADER_X;
                        } else {
                            mspState = MSP_IDLE;
                        }
                        break;
                    case MSP_HEADER_M: // MSPv1 header: $M<
                        if (byte === '<'.charCodeAt(0)) {
                            mspState = MSP_SIZE;
                            // For v1 we'll initialize checksum when size read
                        } else {
                            mspState = MSP_IDLE;
                        }
                        break;
                    case MSP_HEADER_X: // MSPv2 header: $X< or $X>
                        // The byte here is the direction character ('<' or '>')
                        mspCrc = 0; // Initialize CRC for MSPv2
                        mspCrc = crc8_dvb_s2(mspCrc, byte); // Include direction char in CRC
                        mspState = MSP_FLAGS;
                        break;
                    case MSP_FLAGS:
                        // read flags for MSPv2 (not used but included in CRC)
                        mspCrc = crc8_dvb_s2(mspCrc, byte);
                        mspState = MSP_SIZE;
                        break;
                    case MSP_SIZE:
                        mspExpectedSize = byte;
                        if (!isMspV2) {
                            mspChecksum = byte; // init checksum for v1
                            mspState = MSP_COMMAND;
                        } else {
                            mspCrc = crc8_dvb_s2(mspCrc, byte);
                            mspState = MSP_SIZE_HIGH;
                        }
                        break;
                    case MSP_SIZE_HIGH:
                        mspExpectedSize |= (byte << 8);
                        mspCrc = crc8_dvb_s2(mspCrc, byte);
                        mspState = MSP_COMMAND;
                        break;
                    case MSP_COMMAND:
                        mspCommand = byte;
                        if (!isMspV2) {
                            mspChecksum ^= byte;
                            mspPayload = [];
                            mspBytesRead = 0;
                            if (mspExpectedSize > 0) mspState = MSP_PAYLOAD;
                            else mspState = MSP_CHECKSUM;
                        } else {
                            mspCrc = crc8_dvb_s2(mspCrc, byte);
                            mspState = MSP_COMMAND_HIGH;
                        }
                        break;
                    case MSP_COMMAND_HIGH:
                        mspCommand |= (byte << 8);
                        mspCrc = crc8_dvb_s2(mspCrc, byte);
                        mspPayload = [];
                        mspBytesRead = 0;
                        if (mspExpectedSize > 0) mspState = MSP_PAYLOAD;
                        else mspState = MSP_CRC;
                        break;
                    case MSP_PAYLOAD:
                        mspPayload.push(byte);
                        if (!isMspV2) {
                            mspChecksum ^= byte;
                        } else {
                            mspCrc = crc8_dvb_s2(mspCrc, byte);
                        }
                        mspBytesRead++;
                        if (mspBytesRead === mspExpectedSize) {
                            mspState = isMspV2 ? MSP_CRC : MSP_CHECKSUM;
                        }
                        break;
                    case MSP_CHECKSUM: // MSPv1
                        if (byte === mspChecksum) {
                            handleIncomingMspData(mspCommand, mspPayload);
                        } else {
                            log.textContent += `MSPv1 Checksum Error! Expected ${mspChecksum}, got ${byte}\n`;
                        }
                        mspState = MSP_IDLE;
                        break;
                    case MSP_CRC: // MSPv2
                        if (byte === mspCrc) {
                            handleIncomingMspData(mspCommand, mspPayload);
                        } else {
                            log.textContent += `MSPv2 CRC Error! Expected ${mspCrc}, got ${byte}\n`;
                        }
                        mspState = MSP_IDLE;
                        break;
                    default:
                        mspState = MSP_IDLE;
                        break;
                }
            }
        }
    } catch (error) {
        log.textContent += `Read error: ${error.message}\n`;
    }
} // Closing for readLoop()


function readFloat(payload, offset) {
    const bytes = payload.slice(offset, offset + 4);
    const buffer = new ArrayBuffer(4);
    const view = new DataView(buffer);
    bytes.forEach((b, i) => view.setUint8(i, b));
    return view.getFloat32(0, true); // true for little-endian
}

function readInt(payload, offset) {
    const bytes = payload.slice(offset, offset + 4);
    const buffer = new ArrayBuffer(4);
    const view = new DataView(buffer);
    bytes.forEach((b, i) => view.setUint8(i, b));
    return view.getInt32(0, true); // true for little-endian
}

function readUint8(payload, offset) {
    return payload[offset];
}

function readUint16(payload, offset) {
    const bytes = payload.slice(offset, offset + 2);
    const buffer = new ArrayBuffer(2);
    const view = new DataView(buffer);
    bytes.forEach((b, i) => view.setUint8(i, b));
    return view.getUint16(0, true); // true for little-endian
}

function readULong(payload, offset) {
    const bytes = payload.slice(offset, offset + 4);
    const buffer = new ArrayBuffer(4);
    const view = new DataView(buffer);
    bytes.forEach((b, i) => view.setUint8(i, b));
    return view.getUint32(0, true); // true for little-endian (unsigned long)
}

// --- handlers for incoming MSP messages (kept as in original, not duplicated) ---
function _handleLiveDataMsp(payload) {
    let offset = 0;

    // Attitude (roll, pitch, yaw - float)
    const roll = readFloat(payload, offset);
    offset += 4;
    const pitch = readFloat(payload, offset);
    offset += 4;
    const yaw = readFloat(payload, offset);
    offset += 4;

    // Status (isArmed, isFailsafeActive, currentFlightMode - bool, bool, uint8_t)
    const isArmed = readUint8(payload, offset) === 1;
    offset += 1;
    const isFailsafeActive = readUint8(payload, offset) === 1;
    offset += 1;
    const currentFlightMode = readUint8(payload, offset); // This is an enum, will need mapping
    offset += 1;

    // Loop Time (unsigned long)
    const loopTimeUs = readULong(payload, offset);
    offset += 4;

    // Motor Outputs (NUM_MOTORS * uint16_t)
    const motorOutputs = [];
    for (let i = 0; i < 4; i++) { // Assuming NUM_MOTORS = 4
        motorOutputs.push(readUint16(payload, offset));
        offset += 2;
    }

    // Receiver Channels (RECEIVER_CHANNEL_COUNT * uint16_t)
    const receiverChannels = [];
    for (let i = 0; i < 16; i++) { // Assuming RECEIVER_CHANNEL_COUNT = 16
        receiverChannels.push(readUint16(payload, offset));
        offset += 2;
    }

    // Update UI elements
    const receiverTabVisible = document.getElementById('receiverTab').style.display === 'block';
    const threeDViewTabVisible = document.getElementById('3dViewTab').style.display === 'block';
    const infoTabVisible = document.getElementById('infoTab').style.display === 'block';

    if (receiverTabVisible) {
        _updateReceiverBars(receiverChannels);
    }

    if (threeDViewTabVisible) {
        _update3DModel({ roll: roll, pitch: pitch, yaw: yaw });
    }

    if (infoTabVisible) {
        const looptimeDisplay = document.getElementById('looptimeDisplay');
        if (looptimeDisplay) {
            looptimeDisplay.textContent = loopTimeUs;
        }

        const failsafeStatusDisplay = document.getElementById('failsafeStatusDisplay');
        if (failsafeStatusDisplay) {
            failsafeStatusDisplay.textContent = isFailsafeActive ? 'Active' : 'Inactive';
        }

        const flightModeDisplay = document.getElementById('flightModeDisplay');
        if (flightModeDisplay) {
            // Map enum value to string
            let flightModeString = 'UNKNOWN';
            if (currentFlightMode === 0) flightModeString = 'ACRO'; // ACRO_MODE
            if (currentFlightMode === 1) flightModeString = 'ANGLE'; // ANGLE_MODE
            flightModeDisplay.textContent = flightModeString;
        }
    }
}

function _handleSettingsDataMsp(payload) {
    let offset = 0;

    // Read setting name
    let settingName = '';
    while (payload[offset] !== 0 && offset < payload.length) {
        settingName += String.fromCharCode(payload[offset]);
        offset++;
    }
    offset++; // Skip null terminator

    // Read setting type
    const settingType = readUint8(payload, offset);
    offset++;

    let settingValue;

    switch (settingType) {
        case 0: // FLOAT
            settingValue = readFloat(payload, offset);
            break;
        case 1: // INT
            settingValue = readInt(payload, offset);
            break;
        case 2: // UINT8
            settingValue = readUint8(payload, offset);
            break;
        case 3: // UINT16
            settingValue = readUint16(payload, offset);
            break;
        case 4: // ULONG
            settingValue = readULong(payload, offset);
            break;
        case 5: // BOOL
            settingValue = readUint8(payload, offset) === 1;
            break;
        case 6: // STRING
            settingValue = new TextDecoder().decode(new Uint8Array(payload.slice(offset)));
            break;
        case 7: // ENUM_IBUS_PROTOCOL
            {
                const ibusProtocol = readUint8(payload, offset);
                settingValue = (ibusProtocol === 0) ? "IBUS" : (ibusProtocol === 1) ? "PPM" : "UNKNOWN";
            }
            break;
        case 8: // ENUM_IMU_PROTOCOL
            {
                const imuProtocol = readUint8(payload, offset);
                settingValue = (imuProtocol === 0) ? "MPU6050" : "UNKNOWN";
            }
            break;
        case 9: // ENUM_IMU_ROTATION
            {
                const imuRotation = readUint8(payload, offset);
                switch (imuRotation) {
                    case 0: settingValue = "NONE"; break;
                    case 1: settingValue = "90_CW"; break;
                    case 2: settingValue = "180_CW"; break;
                    case 3: settingValue = "270_CW"; break;
                    case 4: settingValue = "FLIP"; break;
                    default: settingValue = "UNKNOWN"; break;
                }
            }
            break;
        case 10: // ENUM_DSHOT_MODE
            {
                const dshotMode = readUint8(payload, offset);
                switch (dshotMode) {
                    case 0: settingValue = "DSHOT_OFF"; break;
                    case 1: settingValue = "DSHOT150"; break;
                    case 2: settingValue = "DSHOT300"; break;
                    case 3: settingValue = "DSHOT600"; break;
                    case 4: settingValue = "DSHOT1200"; break;
                    default: settingValue = "UNKNOWN"; break;
                }
            }
            break;
        case 11: // ENUM_LPF_BANDWIDTH
            {
                const lpfBandwidth = readUint8(payload, offset);
                switch (lpfBandwidth) {
                    case 0: settingValue = "LPF_256HZ_N_0MS"; break;
                    case 1: settingValue = "LPF_188HZ_N_2MS"; break;
                    case 2: settingValue = "LPF_98HZ_N_3MS"; break;
                    case 3: settingValue = "LPF_42HZ_N_5MS"; break;
                    case 4: settingValue = "LPF_20HZ_N_10MS"; break;
                    case 5: settingValue = "LPF_10HZ_N_13MS"; break;
                    case 6: settingValue = "LPF_5HZ_N_18MS"; break;
                    default: settingValue = "UNKNOWN"; break;
                }
            }
            break;
        case 12: // ENUM_RX_CHANNEL_MAP
            // This is handled by a dedicated MSP command, so we shouldn't get it here
            settingValue = "RX_CHANNEL_MAP_DATA";
            break;
        default:
            settingValue = "UNKNOWN_TYPE";
            break;
    }

    // Store the received setting
    receivedSettings[settingName] = settingValue;

    // Update the UI element
    const element = document.querySelector(`[data-setting="${settingName}"]`);
    if (element) {
        if (element.type === 'checkbox') {
            element.checked = settingValue;
        } else if (element.tagName === 'SELECT') {
            // Find the option with the matching value
            let optionFound = false;
            for (let i = 0; i < element.options.length; i++) {
                if (element.options[i].value === settingValue) {
                    element.value = settingValue;
                    optionFound = true;
                    break;
                }
            }
            if (!optionFound) {
                console.warn(`Option for value ${settingValue} not found in select for ${settingName}`);
            }
        }
        else {
            element.value = settingValue;
        }
    } else {
        console.warn(`UI element for setting ${settingName} not found.`);
    }

    // Check if all expected settings have been received
    const allSettingsReceived = expectedSettingKeys.every(key => receivedSettings.hasOwnProperty(key));
    if (allSettingsReceived) {
        _createPidSettingsTable();
        enableTabs();
        log.textContent += 'All settings received and UI updated.\n';
    }
}

function _handleVersionDataMsp(payload) {
    let offset = 0;

    // Read Firmware Version
    let firmwareVersion = '';
    while (payload[offset] !== 0 && offset < payload.length) {
        firmwareVersion += String.fromCharCode(payload[offset]);
        offset++;
    }
    offset++; // Skip null terminator

    // Read Build ID
    let buildId = '';
    while (payload[offset] !== 0 && offset < payload.length) {
        buildId += String.fromCharCode(payload[offset]);
        offset++;
    }
    offset++; // Skip null terminator

    const infoContainer = document.getElementById('infoContainer');

    // Handle Firmware Version
    let versionDiv = document.getElementById('firmwareVersion');
    if (!versionDiv) {
        versionDiv = document.createElement('div');
        versionDiv.id = 'firmwareVersion';
        infoContainer.appendChild(versionDiv);
    }
    versionDiv.innerHTML = `<h3>Firmware Version: ${firmwareVersion}</h3>`;

    // Handle Firmware Build ID
    const buildIdDisplay = document.getElementById('buildIdDisplay');
    if (buildIdDisplay && buildId !== undefined) {
        buildIdDisplay.textContent = buildId;
    }
}

function _handleStatusDataMsp(payload) {
    let offset = 0;

    // Loop Time (unsigned long)
    const loopTimeUs = readULong(payload, offset);
    offset += 4;

    // CPU Load (float)
    const cpuLoad = readFloat(payload, offset);
    offset += 4;

    // Battery Voltage (float)
    const voltage = readFloat(payload, offset);
    offset += 4;

    // Current Draw (float)
    const current = readFloat(payload, offset);
    offset += 4;

    // Is Armed (bool)
    const isArmed = readUint8(payload, offset) === 1;
    offset += 1;

    // Is Failsafe Active (bool)
    const isFailsafeActive = readUint8(payload, offset) === 1;
    offset += 1;

    // Current Flight Mode (uint8_t - enum)
    const currentFlightMode = readUint8(payload, offset);
    offset += 1;

    const infoContainer = document.getElementById('infoContainer');
    let statusDiv = document.getElementById('systemStatus');
    if (!statusDiv) {
        statusDiv = document.createElement('div');
        statusDiv.id = 'systemStatus';
        infoContainer.appendChild(statusDiv);
    }

    let flightModeString = 'UNKNOWN';
    if (currentFlightMode === 0) flightModeString = 'ACRO'; // ACRO_MODE
    if (currentFlightMode === 1) flightModeString = 'ANGLE'; // ANGLE_MODE

    const statusHtml = `
        <h3>System Status:</h3>
        <ul>
            <li>Loop Time (us): ${loopTimeUs}</li>
            <li>CPU Load (%): ${cpuLoad.toFixed(2)}</li>
            <li>Battery Voltage (V): ${voltage.toFixed(2)}</li>
            <li>Current Draw (A): ${current.toFixed(2)}</li>
            <li>Is Armed: ${isArmed ? 'Yes' : 'No'}</li>
            <li>Is Failsafe Active: ${isFailsafeActive ? 'Yes' : 'No'}</li>
            <li>Flight Mode: ${flightModeString}</li>
        </ul>
    `;
    statusDiv.innerHTML = statusHtml;

    // Update specific info displays
    const looptimeDisplay = document.getElementById('looptimeDisplay');
    if (looptimeDisplay) {
        looptimeDisplay.textContent = loopTimeUs;
    }
    const failsafeStatusDisplay = document.getElementById('failsafeStatusDisplay');
    if (failsafeStatusDisplay) {
        failsafeStatusDisplay.textContent = isFailsafeActive ? 'Active' : 'Inactive';
    }
    const flightModeDisplay = document.getElementById('flightModeDisplay');
    if (flightModeDisplay) {
        flightModeDisplay.textContent = flightModeString;
    }
}

function _createPidSettingsTable() {
    const pidSettingsContainer = document.getElementById('pidSettingsContainer');
    pidSettingsContainer.innerHTML = ''; // Clear previous content

    const groupedPidSettings = {};
    for (const key in receivedSettings) {
        if (key.startsWith('pid.')) {
            const parts = key.split('.');
            const subCategory = parts.slice(0, -1).join('.'); // e.g., "pid.roll"
            const settingName = parts[parts.length - 1]; // e.g., "kp"

            if (!groupedPidSettings[subCategory]) {
                groupedPidSettings[subCategory] = {};
            }
            groupedPidSettings[subCategory][settingName] = receivedSettings[key];
        }
    }

    for (const subCategory in groupedPidSettings) {
        const subCategoryTitle = document.createElement('h3');
        subCategoryTitle.textContent = subCategory;
        pidSettingsContainer.appendChild(subCategoryTitle);

        const table = document.createElement('table');
        table.innerHTML = '<thead><tr><th>Name</th><th>Value</th></tr></thead>';
        const tbody = document.createElement('tbody');

        for (const key in groupedPidSettings[subCategory]) {
            const row = document.createElement('tr');
            const nameCell = document.createElement('td');
            const valueCell = document.createElement('td');

            nameCell.textContent = key;

            const input = document.createElement('input');
            input.type = 'text'; // Use text to allow for float values from user
            input.value = groupedPidSettings[subCategory][key];
            input.dataset.setting = `${subCategory}.${key}`;

            valueCell.appendChild(input);
            row.appendChild(nameCell);
            row.appendChild(valueCell);
            tbody.appendChild(row);
        }
        table.appendChild(tbody);
        pidSettingsContainer.appendChild(table);
    }
}

function _updateReceiverBars(receiverChannels) {
    const receiverChannelBarsDiv = document.getElementById('receiverChannelBars');
    if (!receiverChannelBarsDiv) return;

    receiverChannelBarsDiv.innerHTML = ''; // Clear previous bars
    receiverChannels.forEach((channelValue, i) => {
        const channelName = receiverChannelNames[i] || `Channel ${i}`;
        const barContainer = document.createElement('div');
        barContainer.className = 'channel-bar-container';

        const label = document.createElement('span');
        label.className = 'channel-label';
        label.textContent = `${channelName}: ${channelValue}`;
        barContainer.appendChild(label);

        const bar = document.createElement('div');
        bar.className = 'channel-bar';
        // Assuming channel values are between 1000 and 2000 for scaling
        const percentage = ((channelValue - 1000) / 1000) * 100;
        bar.style.width = `${Math.max(0, Math.min(100, percentage))}%`;
        barContainer.appendChild(bar);

        receiverChannelBarsDiv.appendChild(barContainer);
    });
}

function _update3DModel(attitude) {
    if (!quadcopter) return;

    const roll = parseFloat(attitude.roll);
    const pitch = parseFloat(attitude.pitch);
    const yaw = parseFloat(attitude.yaw);

    // Create a new Euler angle object with the corrected rotations
    const euler = new THREE.Euler(
        THREE.MathUtils.degToRad(-pitch),
        THREE.MathUtils.degToRad(yaw),
        THREE.MathUtils.degToRad(-roll),
        'YXZ'
    );

    // Convert the Euler angles to a quaternion and set it as the target
    targetQuaternion.setFromEuler(euler);

    // Update numerical display with the RAW values
    const attitudeDisplay = document.getElementById('attitude-display');
    if (attitudeDisplay) {
        attitudeDisplay.innerHTML = `Roll: ${!isNaN(roll) ? roll.toFixed(2) : 'N/A'}&deg; | Pitch: ${!isNaN(pitch) ? pitch.toFixed(2) : 'N/A'}&deg; | Yaw: ${!isNaN(yaw) ? yaw.toFixed(2) : 'N/A'}&deg;`;
    }
}





function handleIncomingMspData(command, payload) {
    log.textContent += `Received MSP command 0x${command.toString(16)} with payload size ${payload.length}\n`;
    log.scrollTop = log.scrollHeight;

    switch (command) {
        case MSP_FC_SETTING_RESPONSE:
            _handleSettingsDataMsp(payload);
            break;
        case MSP_FC_ERROR:
            {
                const errorMessage = new TextDecoder().decode(new Uint8Array(payload));
                log.textContent += `MSP_FC_ERROR: ${errorMessage}\n`;
            }
            break;
        case MSP_FC_LIVE_DATA:
            _handleLiveDataMsp(payload);
            break;
        case MSP_FC_GET_VERSION: // Response to GET_VERSION
            _handleVersionDataMsp(payload);
            break;
        case MSP_FC_GET_STATUS: // Response to GET_STATUS
            _handleStatusDataMsp(payload);
            break;
        case MSP_FC_DEBUG_MESSAGE:
            {
                const debugMessage = new TextDecoder().decode(new Uint8Array(payload));
                log.textContent += `DEBUG: ${debugMessage}\n`;
            }
            break;
        default:
            log.textContent += `Unhandled MSP Command: 0x${command.toString(16)}\n`;
            break;
    }
}



function initializeSettingEventListeners() {
    // Use event delegation on the document to handle all setting changes
    document.addEventListener('change', (event) => {
        const target = event.target;
        if (target && target.dataset && target.dataset.setting) {
            handleSettingChange(target);
        }
    });
}

function valueToBytes(value, type) {
    const encoder = new TextEncoder();
    let bytes = [];

    switch (type) {
        case 0: // FLOAT
            {
                const buffer = new ArrayBuffer(4);
                const view = new DataView(buffer);
                view.setFloat32(0, parseFloat(value), true); // little-endian
                bytes = Array.from(new Uint8Array(buffer));
            }
            break;
        case 1: // INT
            {
                const buffer = new ArrayBuffer(4);
                const view = new DataView(buffer);
                view.setInt32(0, parseInt(value), true); // little-endian
                bytes = Array.from(new Uint8Array(buffer));
            }
            break;
        case 2: // UINT8
            bytes = [parseInt(value)];
            break;
        case 3: // UINT16
            {
                const buffer = new ArrayBuffer(2);
                const view = new DataView(buffer);
                view.setUint16(0, parseInt(value), true); // little-endian
                bytes = Array.from(new Uint8Array(buffer));
            }
            break;
        case 4: // ULONG
            {
                const buffer = new ArrayBuffer(4);
                const view = new DataView(buffer);
                view.setUint32(0, parseInt(value), true); // little-endian
                bytes = Array.from(new Uint8Array(buffer));
            }
            break;
        case 5: // BOOL
            bytes = [value ? 1 : 0];
            break;
        case 6: // STRING
            bytes = Array.from(encoder.encode(value));
            bytes.push(0); // Null terminator
            break;
        case 7: // ENUM_IBUS_PROTOCOL
            bytes = [value === "IBUS" ? 0 : value === "PPM" ? 1 : 255]; // 0: IBUS, 1: PPM, 255: UNKNOWN
            break;
        case 8: // ENUM_IMU_PROTOCOL
            bytes = [value === "MPU6050" ? 0 : 255]; // 0: MPU6050, 255: UNKNOWN
            break;
        case 9: // ENUM_IMU_ROTATION
            {
                let enumVal = 255; // UNKNOWN
                switch (value) {
                    case "NONE": enumVal = 0; break;
                    case "90_CW": enumVal = 1; break;
                    case "180_CW": enumVal = 2; break;
                    case "270_CW": enumVal = 3; break;
                    case "FLIP": enumVal = 4; break;
                }
                bytes = [enumVal];
            }
            break;
        case 10: // ENUM_DSHOT_MODE
            {
                let enumVal = 255; // UNKNOWN
                switch (value) {
                    case "DSHOT_OFF": enumVal = 0; break;
                    case "DSHOT150": enumVal = 1; break;
                    case "DSHOT300": enumVal = 2; break;
                    case "DSHOT600": enumVal = 3; break;
                    case "DSHOT1200": enumVal = 4; break;
                }
                bytes = [enumVal];
            }
            break;
        case 11: // ENUM_LPF_BANDWIDTH
            {
                let enumVal = 255; // UNKNOWN
                switch (value) {
                    case "LPF_256HZ_N_0MS": enumVal = 0; break;
                    case "LPF_188HZ_N_2MS": enumVal = 1; break;
                    case "LPF_98HZ_N_3MS": enumVal = 2; break;
                    case "LPF_42HZ_N_5MS": enumVal = 3; break;
                    case "LPF_20HZ_N_10MS": enumVal = 4; break;
                    case "LPF_10HZ_N_13MS": enumVal = 5; break;
                    case "LPF_5HZ_N_18MS": enumVal = 6; break;
                }
                bytes = [enumVal];
            }
            break;
        default:
            console.warn(`Unknown setting type for valueToBytes: ${type}`);
            break;
    }
    return bytes;
}

async function handleSettingChange(inputElement) {
    const settingKey = inputElement.dataset.setting;
    let value;
    let settingType;

    if (!writer) {
        log.textContent += 'ERROR: Serial writer not available. Cannot send command.\n';
        return;
    }

    // Determine value and settingType
    if (inputElement.type === 'checkbox') {
        value = inputElement.checked;
        settingType = 5; // BOOL
    } else if (inputElement.tagName === 'SELECT') {
        value = inputElement.value;
        // Infer enum type from settingKey
        if (settingKey.includes('rx.protocol')) settingType = 7; // ENUM_IBUS_PROTOCOL
        else if (settingKey.includes('imu.protocol')) settingType = 8; // ENUM_IMU_PROTOCOL
        else if (settingKey.includes('imu.rotation')) settingType = 9; // ENUM_IMU_ROTATION
        else if (settingKey.includes('motor.dshot_mode')) settingType = 10; // ENUM_DSHOT_MODE
        else if (settingKey.includes('imu.lpf')) settingType = 11; // ENUM_LPF_BANDWIDTH
        else settingType = 255; // Unknown enum type
    } else if (inputElement.type === 'number') {
        value = parseFloat(inputElement.value);
        if (settingKey.includes('kp') || settingKey.includes('ki') || settingKey.includes('kd')) {
            settingType = 1; // INT (PID gains are ints in firmware)
        } else if (settingKey.includes('stages')) {
            settingType = 2; // UINT8
        } else if (settingKey.includes('min') || settingKey.includes('max') || settingKey.includes('threshold')) {
            settingType = 3; // UINT16
        } else if (settingKey.includes('print_interval')) {
            settingType = 4; // ULONG
        }
        else {
            settingType = 0; // FLOAT by default for numbers
        }
    } else if (inputElement.type === 'text') {
        value = inputElement.value;
        if (settingKey.includes('test_string')) { // Assuming this is our test string
            settingType = 6; // STRING
        } else {
            settingType = 255; // Unknown string type
        }
    } else {
        log.textContent += `ERROR: Unhandled input type for setting ${settingKey}.\n`;
        return;
    }

    // Construct MSP payload
    const settingKeyBytes = Array.from(new TextEncoder().encode(settingKey));
    settingKeyBytes.push(0); // Null terminator for string

    const settingTypeByte = [settingType];
    const settingValueBytes = valueToBytes(value, settingType);

    const payload = [...settingKeyBytes, ...settingTypeByte, ...settingValueBytes];

    log.textContent += `Attempting to send MSP_FC_SET_SETTING for ${settingKey} with value ${value}.\n`;
    try {
        await sendMspMessage(MSP_FC_SET_SETTING, payload);
        log.textContent += `MSP_FC_SET_SETTING sent successfully.\n`;
    } catch (error) {
        log.textContent += `ERROR sending MSP_FC_SET_SETTING: ${error.message}\n`;
    }
}

// --- MSP Constants and Functions ---
const MSP_HEADER_V1 = "$M<";
const MSP_HEADER_V2 = "$X<"; // Not used for sending yet, but available

// Custom MSP Commands (must match firmware's MspCommands.h)
const MSP_FC_GET_SETTING = 2000;
const MSP_FC_SET_SETTING = 2001;
const MSP_FC_SETTING_RESPONSE = 2002;
const MSP_FC_ERROR = 2003;
const MSP_FC_GET_RX_MAP = 2004;
const MSP_FC_SET_RX_MAP = 2005;
const MSP_FC_SAVE_SETTINGS = 2006;
const MSP_FC_RESET_SETTINGS = 2007;
const MSP_FC_REBOOT = 2008;
const MSP_FC_GET_STATUS = 2009;
const MSP_FC_GET_VERSION = 2010;
const MSP_FC_CALIBRATE_IMU = 2011;
const MSP_FC_LIVE_DATA = 2012;
const MSP_FC_DEBUG_MESSAGE = 2013;

function calculateChecksum(data) {
    let checksum = 0;
    for (let i = 0; i < data.length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

async function sendMspMessage(command, payload = []) {
    if (!writer) {
        log.textContent += 'ERROR: Serial writer not available. Cannot send MSP command.\n';
        return;
    }

    const size = payload.length;
    let buffer;

    // Determine if MSPv1 or MSPv2 is needed
    // MSPv2 is required if command > 255 or payload size > 255
    const useMspV2 = (command > 255 || size > 255);

    if (useMspV2) {
        // MSPv2 message construction
        let mspCrc = 0;
        const directionChar = '>'.charCodeAt(0); // Host to device
        const flags = 0; // No flags for now

        // Calculate CRC for header
        mspCrc = crc8_dvb_s2(mspCrc, flags);
        mspCrc = crc8_dvb_s2(mspCrc, (size & 0xFF));
        mspCrc = crc8_dvb_s2(mspCrc, (size >> 8));
        mspCrc = crc8_dvb_s2(mspCrc, (command & 0xFF));
        mspCrc = crc8_dvb_s2(mspCrc, (command >> 8));

        // Calculate CRC for payload
        for (let i = 0; i < payload.length; i++) {
            mspCrc = crc8_dvb_s2(mspCrc, payload[i]);
        }

        buffer = new Uint8Array([
            '$'.charCodeAt(0),
            'X'.charCodeAt(0),
            directionChar,
            flags,
            (size & 0xFF),       // Size LSB
            (size >> 8),         // Size MSB
            (command & 0xFF),    // Command LSB
            (command >> 8),      // Command MSB
            ...payload,
            mspCrc
        ]);

    } else {
        // MSPv1 message construction
        const data = [size, command, ...payload];
        const checksum = calculateChecksum(data);

        buffer = new Uint8Array([
            MSP_HEADER_V1.charCodeAt(0),
            MSP_HEADER_V1.charCodeAt(1),
            MSP_HEADER_V1.charCodeAt(2),
            ...data,
            checksum
        ]);
    }

    log.textContent += `Sending MSP command 0x${command.toString(16)} with payload size ${size}.\n`;
    try {
        await writer.write(buffer);
        log.textContent += `MSP command sent successfully.\n`;
    } catch (error) {
        log.textContent += `ERROR sending MSP command: ${error.message}\n`;
    }
}
