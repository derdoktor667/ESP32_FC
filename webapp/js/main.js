const toggleConnectButton = document.getElementById('toggleConnectButton');
const log = document.getElementById('log');
const calibrateImuButton = document.getElementById('calibrateImuButton');

let port;
let writer;
let isConnected = false;

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

    // Quadcopter Model (simple cross)
    quadcopter = new THREE.Group();
    const arm1 = new THREE.Mesh(
        new THREE.BoxGeometry(5, 0.2, 0.2),
        new THREE.MeshStandardMaterial({ color: 0xff0000 })
    );
    const arm2 = new THREE.Mesh(
        new THREE.BoxGeometry(0.2, 0.2, 5),
        new THREE.MeshStandardMaterial({ color: 0x0000ff })
    );
    quadcopter.add(arm1);
    quadcopter.add(arm2);
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
        }
    } else {
        // Disconnect logic
        if (port) {
            try {
                await reader.cancel();
                await writer.close();
                await port.close();

                toggleConnectButton.textContent = 'Connect';
                isConnected = false;

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
    if (data.settings) {
        populateSettings(data.settings);
        enableTabs(); // Enable tabs once settings are received
    }

    if (data.version) {
        const infoContainer = document.getElementById('infoContainer');
        let versionDiv = document.getElementById('firmwareVersion');
        if (!versionDiv) {
            versionDiv = document.createElement('div');
            versionDiv.id = 'firmwareVersion';
            infoContainer.appendChild(versionDiv);
        }
        versionDiv.innerHTML = `<h3>Firmware Version: ${data.version}</h3>`;
    }

    if (data.status) {
        const infoContainer = document.getElementById('infoContainer');
        let statusDiv = document.getElementById('systemStatus');
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

    const threeDViewTab = document.getElementById('3dViewTab');
    if (threeDViewTab.style.display === 'block' && data.live_data && quadcopter) {
        const attitude = data.live_data.attitude;
        
        // Update 3D model
        quadcopter.rotation.x = THREE.MathUtils.degToRad(attitude.pitch);
        quadcopter.rotation.y = THREE.MathUtils.degToRad(attitude.yaw);
        quadcopter.rotation.z = THREE.MathUtils.degToRad(attitude.roll);

        // Update numerical display
        const attitudeDisplay = document.getElementById('attitude-display');
        attitudeDisplay.innerHTML = `Roll: ${attitude.roll.toFixed(2)}&deg; | Pitch: ${attitude.pitch.toFixed(2)}&deg; | Yaw: ${attitude.yaw.toFixed(2)}&deg;`;
    }
}

function populateSettings(settings) {
    const motorSettingsContainer = document.getElementById('motorSettingsContainer');
    const pidSettingsContainer = document.getElementById('pidSettingsContainer');
    const receiverSettingsContainer = document.getElementById('receiverSettingsContainer');

    // Clear previous settings
    motorSettingsContainer.innerHTML = '';
    pidSettingsContainer.innerHTML = '';
    receiverSettingsContainer.innerHTML = '';

    // Group settings by category
    const groupedSettings = {};
    for (const key in settings) {
        const parts = key.split('.');
        const value = settings[key];
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
                input.dataset.category = categoryPrefix;
                input.dataset.key = key;
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

    if (groupedSettings.motor) {
        createSettingsTable(motorSettingsContainer, groupedSettings.motor, 'motor');
    }
    if (groupedSettings.pid) {
        createSettingsTable(pidSettingsContainer, groupedSettings.pid, 'pid');
    }
    if (groupedSettings.rx) {
        createSettingsTable(receiverSettingsContainer, groupedSettings.rx, 'rx');
    }
    // Handle general settings if any, though current structure implies all are categorized
    if (groupedSettings.general) {
        // Decide where to put general settings, for now, let's put them in motor tab as an example
        createSettingsTable(motorSettingsContainer, groupedSettings.general, 'general');
    }
}

async function handleSettingChange(event) {
    const input = event.target;
    const category = input.dataset.category;
    const key = input.dataset.key;
    const value = input.value;

    const command = `set ${category}.${key} ${value}\n`;
    log.textContent += `Sending command: ${command}`;
    await writer.write(new TextEncoder().encode(command));
}