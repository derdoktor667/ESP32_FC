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
let isConnected = false;

// Receiver channel names for display
const receiverChannelNames = [
    "Roll", "Pitch", "Throttle", "Yaw", "Arm", "Failsafe", "Flight Mode",
    "Aux 1", "Aux 2", "Aux 3", "Aux 4", "Aux 5", "Aux 6", "Aux 7", "Aux 8", "Aux 9"
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
    openTab(null, 'infoTab');
    // Find the button that opens the Info tab and add the active class
    const infoButton = Array.from(document.querySelectorAll('.tablinks')).find(btn => btn.textContent === 'Info');
    if (infoButton) {
        infoButton.classList.add('active');
    }
    init3D();
    initializeSettingEventListeners(); // Add this call

    if (calibrateImuButton) {
        calibrateImuButton.addEventListener('click', async () => {
            if (isConnected && writer) {
                log.textContent += 'Sending command: calibrate_imu\n';
                await writer.write(new TextEncoder().encode('calibrate_imu\n'));
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
                log.textContent += 'Sending command: save\n';
                await writer.write(new TextEncoder().encode('save\n'));
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

            // Enter API mode
            await writer.write(new TextEncoder().encode('api\n'));

            // Wait for the device to switch to API mode
            await new Promise(resolve => setTimeout(resolve, 100));

            // Get settings
            await writer.write(new TextEncoder().encode('get_settings\n'));
            // Get version
            await writer.write(new TextEncoder().encode('version\n'));
            // Get status
            await writer.write(new TextEncoder().encode('status\n'));

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
                await reader.cancel();
                reader.releaseLock();
                await writer.close();
                writer.releaseLock();
                await port.close();

                toggleConnectButton.textContent = 'Disconnect';
                isConnected = false;
                saveSettingsButton.disabled = true; // Disable save button on disconnect

                log.textContent += 'Disconnected from device.\n';
                location.reload(); // Reload page after disconnecting
            } catch (error) {
                log.textContent += `Error: ${error.message}\n`;
            }
        }
    }
});

async function readLoop() {
    try {
        let buffer = '';
        while (true) {
            const { value, done } = await reader.read();
            if (done) break;
            buffer += new TextDecoder().decode(value);

            let newlineIndex;
            while ((newlineIndex = buffer.indexOf('\n')) !== -1) {
                const line = buffer.slice(0, newlineIndex).trim();
                buffer = buffer.slice(newlineIndex + 1);

                if (line) {
                    log.textContent += line + '\n';
                    log.scrollTop = log.scrollHeight;

                    if (line.startsWith('{') && line.endsWith('}')) {
                        try {
                            const data = JSON.parse(line);
                            handleIncomingData(data);
                        } catch (e) {
                            console.error("Failed to parse JSON:", line, e);
                        }
                    }
                }
            }
        }
    } catch (error) {
        log.textContent += `Read error: ${error.message}\n`;
    }
}

function _handleSettingsData(settings) {
    try {
        populateSettings(settings);
        enableTabs(); // Enable tabs once settings are received
    } catch (e) {
        console.error("Error populating settings or enabling tabs:", e);
    }
}

function _handleVersionData(data) {
    const infoContainer = document.getElementById('infoContainer');

    // Handle Firmware Version
    let versionDiv = document.getElementById('firmwareVersion');
    if (!versionDiv) {
        versionDiv = document.createElement('div');
        versionDiv.id = 'firmwareVersion';
        infoContainer.appendChild(versionDiv);
    }
    versionDiv.innerHTML = `<h3>Firmware Version: ${data.version}</h3>`; // Use data.version

    // Handle Firmware Build ID
    const buildIdDisplay = document.getElementById('buildIdDisplay');
    if (buildIdDisplay && data.build_id !== undefined) {
        buildIdDisplay.textContent = data.build_id;
    }
}

function _handleStatusData(status) {
    const infoContainer = document.getElementById('infoContainer');
    let statusDiv = document.getElementById('systemStatus');
    if (!statusDiv) {
        statusDiv = document.createElement('div');
        statusDiv.id = 'systemStatus';
        infoContainer.appendChild(statusDiv);
    }
    let statusHtml = '<h3>System Status:</h3><ul>';
    for (const key in status) {
        if (key === 'Loop Time (us)') {
            const looptimeDisplay = document.getElementById('looptimeDisplay');
            if (looptimeDisplay) {
                looptimeDisplay.textContent = status[key];
            }
        }
        statusHtml += `<li>${key}: ${status[key]}</li>`;
    }
    statusHtml += `</ul>`;
    statusDiv.innerHTML = statusHtml;
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


function _handleLiveData(liveData) {
    // Check which tab is active to avoid unnecessary DOM updates
    const receiverTabVisible = document.getElementById('receiverTab').style.display === 'block';
    const threeDViewTabVisible = document.getElementById('3dViewTab').style.display === 'block';
    const infoTabVisible = document.getElementById('infoTab').style.display === 'block';

    if (receiverTabVisible && liveData.receiver_channels) {
        _updateReceiverBars(liveData.receiver_channels);
    }

    if (threeDViewTabVisible && liveData.attitude) {
        _update3DModel(liveData.attitude);
    }

    if (infoTabVisible && liveData.status) {
        const looptimeDisplay = document.getElementById('looptimeDisplay');
        if (looptimeDisplay && liveData.status.loop_time_us) {
            looptimeDisplay.textContent = liveData.status.loop_time_us;
        }

        const failsafeStatusDisplay = document.getElementById('failsafeStatusDisplay');
        if (failsafeStatusDisplay && liveData.status.failsafe !== undefined) {
            failsafeStatusDisplay.textContent = liveData.status.failsafe ? 'Active' : 'Inactive';
        }

        const flightModeDisplay = document.getElementById('flightModeDisplay');
        if (flightModeDisplay && liveData.status.mode) {
            flightModeDisplay.textContent = liveData.status.mode;
        }
    }
}


function handleIncomingData(data) {
    if (!data || typeof data !== 'object') {
        console.warn("Received non-object data or empty data:", data);
        return;
    }

    if (data.settings) {
        _handleSettingsData(data.settings);
    }
    if (data.version) {
        _handleVersionData(data); // Pass the entire data object
    }
    if (data.status) {
        // The firmware sends 'status' for both system status and set command responses.
        // We check for a known key from the system status object to differentiate.
        if ("Loop Time (us)" in data.status) {
            _handleStatusData(data.status);
        }
    }
    if (data.live_data) {
        _handleLiveData(data.live_data);
    }
}

function populateSettings(settings) {
    // Update all settings by iterating through elements with a data-setting attribute
    for (const key in settings) {
        const element = document.querySelector(`[data-setting="${key}"]`);
        if (element) {
            if (element.type === 'checkbox') {
                element.checked = settings[key];
            } else {
                element.value = settings[key];
            }
        }
    }

    // Dynamically create PID settings table (as it's more complex)
    const pidSettingsContainer = document.getElementById('pidSettingsContainer');
    pidSettingsContainer.innerHTML = ''; // Clear previous content

    const groupedPidSettings = {};
    for (const key in settings) {
        if (key.startsWith('pid.')) {
            const parts = key.split('.');
            const subCategory = parts.slice(0, -1).join('.'); // e.g., "pid.roll"
            const settingName = parts[parts.length - 1]; // e.g., "kp"

            if (!groupedPidSettings[subCategory]) {
                groupedPidSettings[subCategory] = {};
            }
            groupedPidSettings[subCategory][settingName] = settings[key];
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

function initializeSettingEventListeners() {
    // Use event delegation on the document to handle all setting changes
    document.addEventListener('change', (event) => {
        const target = event.target;
        if (target && target.dataset.setting) {
            handleSettingChange(target);
        }
    });
}

async function handleSettingChange(inputElement) {
    const settingKey = inputElement.dataset.setting;
    let value;

    if (!writer) {
        log.textContent += 'ERROR: Serial writer not available. Cannot send command.\n';
        return;
    }

    if (inputElement.type === 'checkbox') {
        value = inputElement.checked;
    } else {
        value = inputElement.value;
    }

    const command = `set ${settingKey} ${value}\n`;
    log.textContent += `Attempting to send command: ${command}`;
    try {
        await writer.write(new TextEncoder().encode(command));
        log.textContent += `Command sent successfully.\n`;
    } catch (error) {
        log.textContent += `ERROR sending command: ${error.message}\n`;
    }
}
