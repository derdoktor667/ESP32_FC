import * as THREE from 'three';

const connectButton = document.getElementById('connectButton');
const disconnectButton = document.getElementById('disconnectButton');

const serialOutput = document.getElementById('serialOutput');
const connectionScreen = document.getElementById('connectionScreen');
const mainAppScreen = document.getElementById('mainAppScreen');
const settingsForm = document.getElementById('settingsForm');
const settingsSection = document.querySelector('.settings-section'); // Add this line
const tabButtons = document.querySelectorAll('.tab-button');
const calibrateImuButton = document.getElementById('calibrateImuButton');
const saveAllSettingsButton = document.getElementById('saveAllSettingsButton'); // New button constant

connectButton.addEventListener('click', connectSerial);
disconnectButton.addEventListener('click', disconnectSerial);

calibrateImuButton.addEventListener('click', calibrateImu);
saveAllSettingsButton.addEventListener('click', saveAllSettings); // New event listener

async function saveAllSettings() {
    serialOutput.textContent += 'Attempting to save all settings...\n';
    const settingInputs = settingsForm.querySelectorAll('input'); // Get all inputs within the settings form

    for (const input of settingInputs) {
        const key = input.name;
        let newValue;

        if (input.type === 'number') {
            newValue = parseFloat(input.value);
        } else if (input.type === 'checkbox') {
            newValue = input.checked;
        } else {
            newValue = input.value;
        }

        if (key) { // Ensure key exists
            await sendSerialData(`set ${key} ${newValue}`);
            // Add a small delay to prevent overwhelming the serial buffer
            await new Promise(resolve => setTimeout(resolve, 50));
        }
    }
    serialOutput.textContent += 'All settings save commands sent.\n';
    // Optionally, request settings again to confirm they were saved and re-render
    // sendSerialData('get_settings');
}

let serialPort;
let serialReader;
let keepReading = false;
let abortController; // AbortController instance for serial reads

// Three.js variables
let scene, camera, renderer;
let quadcopterModel;

// --- UI State Management ---
function showScreen(screenId) {
    if (screenId === 'connection') {
        connectionScreen.classList.remove('hidden');
        mainAppScreen.classList.add('hidden');
        settingsSection.classList.add('hidden'); // Hide settings section when connection screen is active
    } else {
        connectionScreen.classList.add('hidden');
        mainAppScreen.classList.remove('hidden');
        settingsSection.classList.remove('hidden'); // Show settings section when main app screen is active
        onWindowResize(); // Ensure canvas is correctly sized after screen transition
    }
}

// --- Serial Communication ---
async function connectSerial() {
    console.log('Connect Serial button clicked!');
    if (!navigator.serial) {
        serialOutput.textContent += 'Error: Web Serial API not supported in this browser or context (e.g., not HTTPS).\n';
        console.error('Web Serial API not supported.');
        return;
    }
    try {
        serialPort = await navigator.serial.requestPort();
        await serialPort.open({ baudRate: 115200 });

        serialOutput.textContent += 'Connected to serial port.\n';
        showScreen('mainApp');
        init3D(); // Initialize 3D scene immediately after showing the main app screen
        window.addEventListener('resize', onWindowResize, false);
        onWindowResize(); // Call once to set initial size

        // Re-get references and attach event listeners for CLI input
        const cliCommandInput = document.getElementById('cliCommandInput');
        const sendCliButton = document.getElementById('sendCliButton');
        if (cliCommandInput && sendCliButton) {
            sendCliButton.addEventListener('click', () => {
                sendSerialData(cliCommandInput.value);
                cliCommandInput.value = ''; // Clear input after sending
            });
        }

        serialReader = serialPort.readable.getReader();
        keepReading = true;
        abortController = new AbortController();
        readSerial(abortController.signal);

        // Send 'api' command to enter API mode
        setTimeout(() => {
            sendSerialData('api');
        }, 500); // Short delay to ensure ESP32 is ready

        // Request all settings after entering API mode
        setTimeout(() => {
            sendSerialData('get_settings');
        }, 1000); // Another short delay

    } catch (error) {
        serialOutput.textContent += `Error connecting: ${error}\n`;
    }
}

async function readSerial(signal) {
    let receivedData = ''; // Buffer for incomplete JSON
    signal.addEventListener('abort', () => {
        console.log('Read operation aborted.');
    });

    while (serialPort.readable && keepReading) {
        try {
            const { value, done } = await serialReader.read({ signal });
            if (done) {
                break;
            }
            const text = new TextDecoder().decode(value);

            receivedData += text;
            // Process each line received
            let lines = receivedData.split('\n');
            receivedData = lines.pop(); // Keep the last (potentially incomplete) line

            for (const line of lines) {
                if (line.trim().length === 0) continue;
                try {
                    const jsonResponse = JSON.parse(line);
                    handleApiResponse(jsonResponse);
                } catch (jsonError) {
                    // Not a JSON object, or malformed JSON, treat as regular serial output
                    serialOutput.textContent += line + '\n'; // Append non-JSON lines to console
                    serialOutput.scrollTop = serialOutput.scrollHeight;
                }
            }

        } catch (error) {
            if (error.name === 'AbortError') {
                serialOutput.textContent += 'Read operation aborted by user.\n';
            } else if (keepReading) {
                serialOutput.textContent += `Error reading: ${error}\n`;
            }
            break; // Exit loop on error or abort
        }
    }
    // Ensure the lock is released when the loop exits
    if (serialReader) {
        serialReader.releaseLock();
        serialReader = undefined;
    }
}

async function disconnectSerial() {
    if (serialPort) {
        try {
            keepReading = false; // Signal the read loop to stop
            if (abortController) {
                abortController.abort(); // Abort any pending read operations
            }
            // Give a moment for the read loop to process the abort signal
            await new Promise(resolve => setTimeout(resolve, 50));

            sendSerialData('reboot');
            await new Promise(resolve => setTimeout(resolve, 50));

            if (serialReader) {
                await serialReader.cancel(); // Cancel any pending read
            }
            await serialPort.close();
            serialPort = undefined;
            serialOutput.textContent += 'Disconnected from serial port.\n';
            connectButton.disabled = false;
            disconnectButton.disabled = true;

            showScreen('connection');

        } catch (error) {
            serialOutput.textContent += `Error disconnecting: ${error}\n`;
        }
    }
}

async function sendSerialData(data) {
    if (serialPort && serialPort.writable) {
        try {
            const writer = serialPort.writable.getWriter();
            const dataArrayBuffer = new TextEncoder().encode(data + '\n');
            await writer.write(dataArrayBuffer);
            writer.releaseLock();
            serialOutput.textContent += `Sent: ${data}\n`;
        } catch (error) {
            serialOutput.textContent += `Error sending: ${error}\n`;
        }
    } else {
        serialOutput.textContent += 'Serial port not connected or not writable.\n';
    }
}

// --- API Response Handling ---
function handleApiResponse(response) {
    console.log('Handling API Response:', response);
    if (response.settings) {
        renderSettings(response.settings);
    } else if (response.live_data) {
        updateQuadcopterModel(response.live_data.attitude.roll, response.live_data.attitude.pitch, response.live_data.attitude.yaw);
    } else if (response.status === 'api_mode_activated') {
        serialOutput.textContent += 'API Mode Activated.\n';
    } else if (response.error) {
        serialOutput.textContent += `API Error: ${response.error}\n`;
    } else {
        // console.log('Unhandled API Response:', response);
    }
}

// --- Settings Management ---
const settingCategories = {
    "pid": ["pid.roll.kp", "pid.roll.ki", "pid.roll.kd", "pid.pitch.kp", "pid.pitch.ki", "pid.pitch.kd", "pid.yaw.kp", "pid.yaw.ki", "pid.yaw.kd", "pid.integral_limit"],
    "rates": ["rates.angle", "rates.yaw", "rates.acro"],
    "receiver": ["rx.min", "rx.max", "rx.arming_threshold", "rx.failsafe_threshold", "rx.protocol", "rx.map.throttle", "rx.map.roll", "rx.map.pitch", "rx.map.yaw", "rx.map.arm_switch", "rx.map.failsafe_switch", "rx.map.flight_mode_switch"],
    "motor": ["motor.idle_speed", "motor.dshot_mode"],
    "filter": ["madgwick.sample_freq", "madgwick.beta"],
    "imu": ["imu.protocol"]
};

function renderSettings(settingsData) {
    // Clear existing content in all tab content areas
    for (const category in settingCategories) {
        document.getElementById(category).innerHTML = '';
    }

    for (const key in settingsData) {
        if (Object.hasOwnProperty.call(settingsData, key)) {
            const value = settingsData[key];

            let targetTabId = 'general'; // Default tab
            for (const category in settingCategories) {
                if (settingCategories[category].includes(key)) {
                    targetTabId = category;
                    break;
                }
            }
            const targetTab = document.getElementById(targetTabId);
            if (!targetTab) continue; // Skip if tab not found

            const settingDiv = document.createElement('div');
            settingDiv.classList.add('setting-item');

            const label = document.createElement('label');
            label.textContent = key + ':';
            label.htmlFor = key.replace(/\./g, '_');

            const input = document.createElement('input');
            input.id = key.replace(/\./g, '_');
            input.name = key;

            if (typeof value === 'number') {
                input.type = 'number';
                input.value = value;
                input.step = 'any';
            } else if (typeof value === 'boolean') {
                input.type = 'checkbox';
                input.checked = value;
            } else {
                input.type = 'text';
                input.value = value;
            }

            const saveButton = document.createElement('button');
            saveButton.textContent = 'Save';
            saveButton.onclick = () => {
                let newValue;
                if (input.type === 'number') {
                    newValue = parseFloat(input.value);
                } else if (input.type === 'checkbox') {
                    newValue = input.checked;
                } else {
                    newValue = input.value;
                }
                sendSerialData(`set ${key} ${newValue}`);
            };

            settingDiv.appendChild(label);
            settingDiv.appendChild(input);
            settingDiv.appendChild(saveButton);
            targetTab.appendChild(settingDiv);
        }
    }
}

// --- Tab Management ---
tabButtons.forEach(button => {
    button.addEventListener('click', () => {
        const tabId = button.dataset.tab;

        // Deactivate all tab buttons and hide all tab contents
        tabButtons.forEach(btn => btn.classList.remove('active'));
        document.querySelectorAll('.tab-content').forEach(content => content.classList.add('hidden'));

        // Activate the clicked tab button and show its content
        button.classList.add('active');
        document.getElementById(tabId).classList.remove('hidden');
    });
});

// --- Three.js 3D Model ---

// Initial screen setup
showScreen('connection');



function init3D() {
    const canvas = document.getElementById('quadcopterCanvas');
    if (!canvas) {
        console.error('Quadcopter canvas not found!');
        return;
    }

    try {
        scene = new THREE.Scene();
        camera = new THREE.PerspectiveCamera(75, canvas.clientWidth / canvas.clientHeight, 0.1, 1000);
        renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true, alpha: true });
        renderer.setSize(canvas.clientWidth, canvas.clientHeight);
        renderer.setClearColor(0x000000, 0); // Transparent background

        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        scene.add(ambientLight);

        // Add directional light
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(0, 1, 1).normalize();
        scene.add(directionalLight);



        // Create quadcopter model using primitives
        quadcopterModel = new THREE.Group();

                // Body
                const bodyGeometry = new THREE.BoxGeometry(0.5, 0.08, 0.3); // Flatter and slightly wider
                const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 }); // Dark grey body
                const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
                quadcopterModel.add(body);
            
                // Orientation Marker (Front)
                const markerGeometry = new THREE.BoxGeometry(0.05, 0.05, 0.05);
                const markerMaterial = new THREE.MeshPhongMaterial({ color: 0xff0000 }); // Bright red
                const marker = new THREE.Mesh(markerGeometry, markerMaterial);
                marker.position.set(0, 0.05, 0.15); // Position on top-front of the body
                quadcopterModel.add(marker);        
            // Arms (more realistic shape and position)
            const armMaterial = new THREE.MeshPhongMaterial({ color: 0x555555 }); // Slightly lighter grey arms
        
            // Front-right arm
            const armFRGeometry = new THREE.BoxGeometry(0.4, 0.03, 0.03); // Thinner, longer
            const armFR = new THREE.Mesh(armFRGeometry, armMaterial);
            armFR.position.set(0.2, 0, 0.15); // Positioned relative to body center
            armFR.rotation.y = Math.PI / 4; // Angle outwards
            quadcopterModel.add(armFR);
        
            // Back-left arm
            const armBLGeometry = new THREE.BoxGeometry(0.4, 0.03, 0.03);
            const armBL = new THREE.Mesh(armBLGeometry, armMaterial);
            armBL.position.set(-0.2, 0, -0.15);
            armBL.rotation.y = Math.PI / 4; // Angle outwards
            quadcopterModel.add(armBL);
        
            // Front-left arm
            const armFLGeometry = new THREE.BoxGeometry(0.4, 0.03, 0.03);
            const armFL = new THREE.Mesh(armFLGeometry, armMaterial);
            armFL.position.set(-0.2, 0, 0.15);
            armFL.rotation.y = -Math.PI / 4; // Angle inwards
            quadcopterModel.add(armFL);
        
            // Back-right arm
            const armBRGeometry = new THREE.BoxGeometry(0.4, 0.03, 0.03);
            const armBR = new THREE.Mesh(armBRGeometry, armMaterial);
            armBR.position.set(0.2, 0, -0.15);
            armBR.rotation.y = -Math.PI / 4; // Angle inwards
            quadcopterModel.add(armBR);
            // Motor and Propeller positions
            const motorPropPositions = [
                { x: 0.3, z: 0.25, armRotation: Math.PI / 4 },  // Front Right
                { x: -0.3, z: -0.25, armRotation: Math.PI / 4 }, // Back Left
                { x: 0.3, z: -0.25, armRotation: -Math.PI / 4 },  // Back Right
                { x: -0.3, z: 0.25, armRotation: -Math.PI / 4 }   // Front Left
            ];
        
            const motorGeometry = new THREE.CylinderGeometry(0.04, 0.04, 0.05, 16);
            const motorMaterial = new THREE.MeshPhongMaterial({ color: 0x222222 }); // Dark grey motors
        
            const propellerGeometry = new THREE.BoxGeometry(0.15, 0.005, 0.03); // Flat propeller blade
            const propellerMaterial = new THREE.MeshPhongMaterial({ color: 0x444444 }); // Darker grey propellers
        
            motorPropPositions.forEach(pos => {
                // Motor
                const motor = new THREE.Mesh(motorGeometry, motorMaterial);
                motor.position.set(pos.x, -0.025, pos.z); // Position below arm
                quadcopterModel.add(motor);
        
                // Propeller
                const prop = new THREE.Mesh(propellerGeometry, propellerMaterial);
                prop.position.set(pos.x, 0.02, pos.z); // Position above motor
                prop.rotation.y = pos.armRotation; // Align with arm
                quadcopterModel.add(prop);
        
                const prop2 = new THREE.Mesh(propellerGeometry, propellerMaterial);
                prop2.position.set(pos.x, 0.02, pos.z); // Position above motor
                prop2.rotation.y = pos.armRotation + Math.PI / 2; // Second blade perpendicular
                quadcopterModel.add(prop2);
            });
        
            // Landing Gear
            const landingGearMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 }); // Grey landing gear
        
            // Front leg
            const legFGeometry = new THREE.BoxGeometry(0.02, 0.1, 0.02);
            const legF = new THREE.Mesh(legFGeometry, landingGearMaterial);
            legF.position.set(0.15, -0.08, 0.1);
            quadcopterModel.add(legF);
        
            // Back leg
            const legBGeometry = new THREE.BoxGeometry(0.02, 0.1, 0.02);
            const legB = new THREE.Mesh(legBGeometry, landingGearMaterial);
            legB.position.set(-0.15, -0.08, -0.1);
            quadcopterModel.add(legB);
        
            // Crossbar
            const crossbarGeometry = new THREE.BoxGeometry(0.35, 0.02, 0.02);
            const crossbar = new THREE.Mesh(crossbarGeometry, landingGearMaterial);
            crossbar.position.set(0, -0.12, 0);
            quadcopterModel.add(crossbar);
        scene.add(quadcopterModel);

        // Add local axes to the quadcopter model for debugging orientation
        const axes = new THREE.AxesHelper(0.3); // Smaller axes for the model itself
        quadcopterModel.add(axes);

        camera.position.z = 2; // Adjust camera position to see the model

        animate();
    } catch (e) {
        console.error('Error during 3D scene initialization:', e);
    }
}

function animate() {
    requestAnimationFrame(animate);
    if (renderer && scene && camera) {
        renderer.render(scene, camera);
    } else {
        // console.warn('Renderer, scene, or camera not defined for animation.');
    }
}

function onWindowResize() {
    const canvas = document.getElementById('quadcopterCanvas');
    if (canvas && camera && renderer) {
        camera.aspect = canvas.clientWidth / canvas.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    }
}

function updateQuadcopterModel(roll, pitch, yaw) {
    // console.log(`Updating 3D model: Roll=${roll}, Pitch=${pitch}, Yaw=${yaw}`);
    if (!scene || !quadcopterModel) {
        // console.warn('3D scene or quadcopter model not yet initialized.');
        return;
    }

    if (quadcopterModel) {
        const rollRad = THREE.MathUtils.degToRad(roll);
        const pitchRad = THREE.MathUtils.degToRad(pitch);
        const yawRad = THREE.MathUtils.degToRad(yaw);
        // console.log(`Applying rotation: Roll=${rollRad}, Pitch=${pitchRad}, Yaw=${yawRad}`);
        quadcopterModel.rotation.set(rollRad, pitchRad, yawRad, 'YXZ');
    } else {
        // console.warn('Quadcopter model not found in scene.');
    }
}

async function calibrateImu() {
    sendSerialData('calibrate');
    if (quadcopterModel) {
        quadcopterModel.rotation.set(0, 0, 0);
    }
}
