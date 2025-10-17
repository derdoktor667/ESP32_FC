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

const toggleConnectButton = document.getElementById('toggleConnectButton');
const log = document.getElementById('log');
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

function init3D() {
    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);

    // Camera
    camera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000); // Initial aspect ratio
    camera.position.z = 10; // Move camera further away

    // Lighting
    const ambientLight = new THREE.AmbientLight(0x404040);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
    directionalLight.position.set(1, 1, 1);
    scene.add(directionalLight);

    // Quadcopter Model
    quadcopter = new THREE.Group();

    // Central Body (e.g., a larger box)
    const bodyGeometry = new THREE.BoxGeometry(2, 0.5, 2);
    const bodyMaterial = new THREE.MeshStandardMaterial({ color: 0x333333 });
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
    quadcopter.add(body);

    // Arms (4 arms extending from the body)
    const armGeometry = new THREE.BoxGeometry(0.2, 0.2, 3);
    const armMaterial = new THREE.MeshStandardMaterial({ color: 0x666666 });

    const arm1 = new THREE.Mesh(armGeometry, armMaterial); // Front-Right
    arm1.position.set(1, 0, 1);
    arm1.rotation.y = Math.PI / 4; // Rotate to align with diagonal
    quadcopter.add(arm1);

    const arm2 = new THREE.Mesh(armGeometry, armMaterial); // Front-Left
    arm2.position.set(-1, 0, 1);
    arm2.rotation.y = -Math.PI / 4; // Rotate to align with diagonal
    quadcopter.add(arm2);

    const arm3 = new THREE.Mesh(armGeometry, armMaterial); // Rear-Left
    arm3.position.set(-1, 0, -1);
    arm3.rotation.y = Math.PI / 4; // Rotate to align with diagonal
    quadcopter.add(arm3);

    const arm4 = new THREE.Mesh(armGeometry, armMaterial); // Rear-Right
    arm4.position.set(1, 0, -1);
    arm4.rotation.y = -Math.PI / 4; // Rotate to align with diagonal
    quadcopter.add(arm4);

    // Propellers (simple cylinders at the end of each arm)
    const propGeometry = new THREE.CylinderGeometry(0.5, 0.5, 0.1, 32);
    const propMaterial = new THREE.MeshStandardMaterial({ color: 0x0000ff });

    const prop1 = new THREE.Mesh(propGeometry, propMaterial);
    prop1.position.set(2.5, 0.2, 2.5); // Position relative to arm end
    quadcopter.add(prop1);

    const prop2 = new THREE.Mesh(propGeometry, propMaterial);
    prop2.position.set(-2.5, 0.2, 2.5);
    quadcopter.add(prop2);

    const prop3 = new THREE.Mesh(propGeometry, propMaterial);
    prop3.position.set(-2.5, 0.2, -2.5);
    quadcopter.add(prop3);

    const prop4 = new THREE.Mesh(propGeometry, propMaterial);
    prop4.position.set(2.5, 0.2, -2.5);
    quadcopter.add(prop4);

    // Flight Direction Marker (a small red box on the front of the body)
    const markerGeometry = new THREE.BoxGeometry(0.5, 0.2, 0.2);
    const markerMaterial = new THREE.MeshStandardMaterial({ color: 0xff0000 });
    const marker = new THREE.Mesh(markerGeometry, markerMaterial);
    marker.position.set(0, 0.25, 1.1); // Position on the front of the central body
    quadcopter.add(marker);

    scene.add(quadcopter);

    animate();
}

function animate() {
    requestAnimationFrame(animate);
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

    if (calibrateImuButton) {
        calibrateImuButton.addEventListener('click', async () => {
            if (isConnected && writer) {
                log.textContent += 'Sending command: calibrate_imu\n';
                await writer.write(new TextEncoder().encode('calibrate_imu\n'));
                // Reset 3D model orientation
                if (quadcopter) {
                    quadcopter.rotation.x = 0;
                    quadcopter.rotation.y = 0;
                    quadcopter.rotation.z = 0;
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

                toggleConnectButton.textContent = 'Connect';
                isConnected = false;
                saveSettingsButton.disabled = true; // Disable save button on disconnect

                log.textContent += 'Disconnected from device.\n';
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

function handleIncomingData(data) {
    if (data && typeof data === 'object') {
        if (data.settings) {
            try {
                populateSettings(data.settings);
                enableTabs(); // Enable tabs once settings are received
            } catch (e) {
                console.error("Error populating settings or enabling tabs:", e);
            }
        }

        if (data.version) {
            const infoContainer = document.getElementById('infoContainer');
            let versionDiv = document.createElement('div');
            if (!versionDiv) {
                versionDiv = document.createElement('div');
                versionDiv.id = 'firmwareVersion';
                infoContainer.appendChild(versionDiv);
            }
            versionDiv.innerHTML = `<h3>Firmware Version: ${data.version}</h3>`;
        }

        if (data.status) {
            const infoContainer = document.getElementById('infoContainer');
            let statusDiv = document.createElement('div');
            if (!statusDiv) {
                statusDiv = document.createElement('div');
                statusDiv.id = 'systemStatus';
                infoContainer.appendChild(statusDiv);
            }
            let statusHtml = '<h3>System Status:</h3>';
            statusHtml += `<ul>`;
            for (const key in data.status) {
                statusHtml += `<li>${key}: ${data.status[key]}</li>`;
            }
            statusHtml += `</ul>`;
            statusDiv.innerHTML = statusHtml;
        }

        // Update receiver live data bars
        const receiverTab = document.getElementById('receiverTab');
        if (receiverTab && receiverTab.style.display === 'block' && data.live_data && data.live_data.receiver_channels) {
            const receiverChannels = data.live_data.receiver_channels;
            const receiverChannelBarsDiv = document.getElementById('receiverChannelBars');
            if (receiverChannelBarsDiv) {
                receiverChannelBarsDiv.innerHTML = ''; // Clear previous bars
                for (let i = 0; i < receiverChannels.length; i++) {
                    const channelName = receiverChannelNames[i] || `Channel ${i}`;
                    const channelValue = receiverChannels[i];
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
                }
            }
        }

        const threeDViewTab = document.getElementById('3dViewTab');
        if (threeDViewTab && threeDViewTab.style.display === 'block' && data.live_data && data.live_data.attitude && quadcopter) {
            const attitude = data.live_data.attitude;
            const roll = parseFloat(attitude.roll);
            const pitch = parseFloat(attitude.pitch);
            const yaw = parseFloat(attitude.yaw);
            
            // Update 3D model
            if (!isNaN(pitch)) quadcopter.rotation.x = THREE.MathUtils.degToRad(pitch);
            if (!isNaN(yaw)) quadcopter.rotation.y = THREE.MathUtils.degToRad(yaw);
            if (!isNaN(roll)) quadcopter.rotation.z = THREE.MathUtils.degToRad(roll);

            // Update numerical display
            const attitudeDisplay = document.getElementById('attitude-display');
            if (attitudeDisplay) {
                attitudeDisplay.innerHTML = `Roll: ${!isNaN(roll) ? roll : 'N/A'}&deg; | Pitch: ${!isNaN(pitch) ? pitch : 'N/A'}&deg; | Yaw: ${!isNaN(yaw) ? yaw : 'N/A'}&deg;`;
            }
        }
    } else {
        console.warn("Received non-object data or empty data:", data);
    }
}

function populateSettings(settings) {
    const motorSettingsContainer = document.getElementById('motorSettingsContainer');
    const pidSettingsContainer = document.getElementById('pidSettingsContainer');
    const receiverSettingsContainer = document.getElementById('receiverSettingsContainer');

    // Clear previous settings
    // motorSettingsContainer.innerHTML = ''; // Now static content
    pidSettingsContainer.innerHTML = '';
    // receiverSettingsContainer.innerHTML = ''; // Now static content

    // Update filter settings UI directly
    updateFilterSettingsUI(settings);
    // Update IMU settings UI directly
    updateImuSettingsUI(settings);
    // Update motor settings UI directly
    updateMotorSettingsUI(settings);
    // Update receiver settings UI directly
    updateReceiverSettingsUI(settings);

    // Group settings by category
    const groupedSettings = {};
    for (const key in settings) {
        const parts = key.split('.');
        const value = settings[key];
        // Skip filter, IMU, motor, and receiver settings as they are handled by dedicated update functions
        if (parts[0] === 'filter' || parts[0] === 'gyro' || parts[0] === 'accel' || parts[0] === 'imu' || parts[0] === 'motor' || parts[0] === 'rx') {
            continue;
        }
        if (parts.length > 1) {
            const category = parts[0]; // Use the first part for the main category
            const subCategory = parts.slice(0, -1).join('.'); // Keep full category for display
            const settingName = parts[parts.length - 1];

            if (!groupedSettings[category]) {
                groupedSettings[category] = {};
            }
            if (!groupedSettings[category][subCategory]) {
                groupedSettings[category][subCategory] = {};
            }
            groupedSettings[category][subCategory][settingName] = value;
        } else {
            // Handle settings without a category (e.g., top-level settings)
            if (!groupedSettings['general']) {
                groupedSettings['general'] = {};
            }
            if (!groupedSettings['general']['general']) {
                groupedSettings['general']['general'] = {};
            }
            groupedSettings['general']['general'][key] = value;
        }
    }

    // Function to create and append settings table to a container
    const createSettingsTable = (container, categoryData, categoryPrefix) => {
        for (const subCategory in categoryData) {
            const subCategoryTitle = document.createElement('h3');
            subCategoryTitle.textContent = subCategory.charAt(0).toUpperCase() + subCategory.slice(1);
            container.appendChild(subCategoryTitle);

            const table = document.createElement('table');
            const thead = document.createElement('thead');
            const tbody = document.createElement('tbody');

            thead.innerHTML = '<tr><th>Name</th><th>Value</th></tr>';
            table.appendChild(thead);

            for (const key in categoryData[subCategory]) {
                const row = document.createElement('tr');
                const nameCell = document.createElement('td');
                const valueCell = document.createElement('td');

                nameCell.textContent = key;
                
                const input = document.createElement('input');
                input.type = 'text';
                input.value = categoryData[subCategory][key];
                input.dataset.setting = `${subCategory}.${key}`;
                input.addEventListener('change', handleSettingChange);

                valueCell.appendChild(input);
                row.appendChild(nameCell);
                row.appendChild(valueCell);
                tbody.appendChild(row);
            }

            table.appendChild(tbody);
            container.appendChild(table);
        }
    };

    if (groupedSettings.pid) {
        createSettingsTable(pidSettingsContainer, groupedSettings.pid, 'pid');
    }
    // Handle general settings if any, though current structure implies all are categorized
    if (groupedSettings.general) {
        // Decide where to put general settings, for now, let's put them in motor tab as an example
        createSettingsTable(motorSettingsContainer, groupedSettings.general, 'general');
    }
}

async function handleSettingChange(event) {
    const input = event.target;
    const settingKey = input.dataset.setting; // Use data-setting attribute for full key
    console.log("handleSettingChange called for:", input.id, settingKey);
    let value;

    if (input.type === 'checkbox') {
        value = input.checked;
    } else {
        value = input.value;
    }

    const command = `set ${settingKey} ${value}\n`;
    log.textContent += `Sending command: ${command}`;
    await writer.write(new TextEncoder().encode(command));
}

function updateFilterSettingsUI(settings) {
    // Gyroscope Low-Pass Filter
    document.getElementById('gyroLpfCutoffFreq').value = settings['gyro.lpf_cutoff_freq'];
    document.getElementById('gyroLpfStages').value = settings['gyro.lpf_stages'];

    // Accelerometer Low-Pass Filter
    document.getElementById('accelLpfCutoffFreq').value = settings['accel.lpf_cutoff_freq'];
    document.getElementById('accelLpfStages').value = settings['accel.lpf_stages'];

    // Complementary Filter
    document.getElementById('complementaryFilterTau').value = settings['filter.comp_tau'];

    // Gyroscope Notch Filter
    document.getElementById('enableGyroNotchFilter').checked = settings['gyro.notch.enable'];
    document.getElementById('gyroNotchFreq').value = settings['gyro.notch.freq'];
    document.getElementById('gyroNotchQ').value = settings['gyro.notch.q'];

    // Add event listeners
    document.getElementById('gyroLpfCutoffFreq').addEventListener('change', handleSettingChange);
    document.getElementById('gyroLpfStages').addEventListener('change', handleSettingChange);
    document.getElementById('accelLpfCutoffFreq').addEventListener('change', handleSettingChange);
    document.getElementById('accelLpfStages').addEventListener('change', handleSettingChange);
    document.getElementById('complementaryFilterTau').addEventListener('change', handleSettingChange);
    document.getElementById('enableGyroNotchFilter').addEventListener('change', handleSettingChange);
    document.getElementById('gyroNotchFreq').addEventListener('change', handleSettingChange);
    document.getElementById('gyroNotchQ').addEventListener('change', handleSettingChange);
}

function updateImuSettingsUI(settings) {
    // IMU Protocol
    document.getElementById('imuProtocol').value = settings['imu.protocol'];
    // IMU LPF Bandwidth
    document.getElementById('imuLpfBandwidth').value = settings['imu.lpf'];
    // IMU Rotation
    document.getElementById('imuRotation').value = settings['imu.rotation'];

    // Add event listeners
    document.getElementById('imuProtocol').addEventListener('change', handleSettingChange);
    document.getElementById('imuLpfBandwidth').addEventListener('change', handleSettingChange);
    document.getElementById('imuRotation').addEventListener('change', handleSettingChange);
}

function updateMotorSettingsUI(settings) {
    // Motor Idle Speed
    document.getElementById('motorIdleSpeedPercent').value = settings['motor.idle_speed'];
    // DShot Mode
    document.getElementById('dshotMode').value = settings['motor.dshot_mode'];

    // Add event listeners
    document.getElementById('motorIdleSpeedPercent').addEventListener('change', handleSettingChange);
    document.getElementById('dshotMode').addEventListener('change', handleSettingChange);
}

function updateReceiverSettingsUI(settings) {
    // Receiver Protocol
    document.getElementById('receiverProtocol').value = settings['rx.protocol'];

    // Channel Mapping
    document.getElementById('rxMapThrottle').value = settings['rx.map.throttle'];
    document.getElementById('rxMapRoll').value = settings['rx.map.roll'];
    document.getElementById('rxMapPitch').value = settings['rx.map.pitch'];
    document.getElementById('rxMapYaw').value = settings['rx.map.yaw'];
    document.getElementById('rxMapArmSwitch').value = settings['rx.map.arm_switch'];
    document.getElementById('rxMapFailsafeSwitch').value = settings['rx.map.failsafe_switch'];
    document.getElementById('rxMapFlightModeSwitch').value = settings['rx.map.flight_mode_switch'];

    // Add event listeners
    document.getElementById('receiverProtocol').addEventListener('change', handleSettingChange);
    document.getElementById('rxMapThrottle').addEventListener('change', handleSettingChange);
    document.getElementById('rxMapRoll').addEventListener('change', handleSettingChange);
    document.getElementById('rxMapPitch').addEventListener('change', handleSettingChange);
    document.getElementById('rxMapYaw').addEventListener('change', handleSettingChange);
    document.getElementById('rxMapArmSwitch').addEventListener('change', handleSettingChange);
    document.getElementById('rxMapFailsafeSwitch').addEventListener('change', handleSettingChange);
    document.getElementById('rxMapFlightModeSwitch').addEventListener('change', handleSettingChange);
}
