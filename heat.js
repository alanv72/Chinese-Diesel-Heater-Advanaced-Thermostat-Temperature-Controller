const ML_TO_GALLON = 0.000264172;
let fanControlStates = { duct: false, wall: false };
let isSliderActive = false;
let tempAdjusting = false;
let pendingSetTemp = null;
let isShuttingDown = false;
let isEditingBLEName = false;

const QUICK_SET_TEMPS = {
  "Frost": 46,
  "Eco": 61,
  "Sleep": 66,
  "Comfort": 72
};

// Global storage for chart data
let tempHistoryData = { values: [], timestamps: [] };
let outsideTempHistoryData = { values: [], timestamps: [] };
let pumpHzHistoryData = { values: [], timestamps: [] };
let voltageHistoryData = { values: [], timestamps: [] };
let ampsHistoryData = { values: [], timestamps: [] };
let hourlyFuelHistoryData = { completed: [], current: null, timestamps: [], accumulator: 0 };
let wattHourHistoryData = { completed: [], current: null, timestamps: [], accumulator: 0 };

// Global chart variables
let tempChart = null;
let voltageChart = null;
let hourlyFuelChart = null;
let wattHourChart = null;

// Reusable alignment function
const alignData = (filteredValues, filteredTimestamps, masterTimestamps) => {
  const data = new Array(masterTimestamps.length).fill(null);
  filteredTimestamps.forEach((timestamp, index) => {
    const idx = masterTimestamps.indexOf(timestamp);
    if (idx !== -1) data[idx] = filteredValues[index];
  });
  return data;
};

function drawGauge(canvasId, percent) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) {
    console.error(`Canvas ${canvasId} not found`);
    return;
  }
  const ctx = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  const centerX = width / 2;
  const centerY = height * 0.75;
  const lineWidth = 19.8;
  const radius = Math.min(width, height) * 0.4 - lineWidth / 2;
  const startAngle = Math.PI;
  const maxAngle = Math.PI;

  ctx.clearRect(0, 0, width, height);
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, startAngle, startAngle + maxAngle);
  ctx.lineWidth = lineWidth;
  ctx.strokeStyle = '#555';
  ctx.stroke();
  const fillAngle = (percent / 100) * Math.PI;
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, startAngle, startAngle + fillAngle);
  const gradient = ctx.createLinearGradient(centerX - radius, centerY, centerX + radius, centerY);
  gradient.addColorStop(0, '#FFA500');
  gradient.addColorStop(1, '#FF4500');
  ctx.lineWidth = lineWidth;
  ctx.strokeStyle = gradient;
  ctx.stroke();
}

function updateFanSpeed(fanType, percent) {
  percent = Math.round(parseFloat(percent) / 5) * 5;
  if (percent < 0 || percent > 100) return;

  drawGauge(fanType + 'FanGauge', percent);
  document.getElementById(fanType + 'FanGaugeValue').textContent = percent + '%';

  if (fanControlStates[fanType]) {
    fetch('/setFanSpeed', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `${fanType}Speed=${percent}`
    })
    .then(response => response.text())
    .then(data => console.log(data))
    .catch(error => console.error('Error:', error));
  }
}

function handleGaugeInteraction(fanType, event) {
  if (!fanControlStates[fanType]) {
    console.log(`${fanType} gauge: Not in manual mode, ignoring click`);
    return;
  }

  const canvas = document.getElementById(fanType + 'FanGauge');
  const rect = canvas.getBoundingClientRect();
  const centerX = canvas.width / 2;
  const centerY = canvas.height * 0.75;
  const x = (event.clientX - rect.left) * (canvas.width / rect.width) - centerX;
  const y = (event.clientY - rect.top) * (canvas.height / rect.height) - centerY;

  const lineWidth = 19.8;
  const radius = Math.min(canvas.width, canvas.height) * 0.4 - lineWidth / 2;
  const distance = Math.sqrt(x * x + y * y);

  if (distance > radius * 1.5) return;

  let angle = Math.atan2(y, x);
  if (angle < 0) angle += 2 * Math.PI;
  angle = (angle > Math.PI) ? (angle - Math.PI) : (Math.PI - angle);
  angle = Math.max(0, Math.min(Math.PI, angle));
  const percent = Math.round((angle / Math.PI) * 100);

  updateFanSpeed(fanType, percent);
}

function toggleFanControl(fanType) {
  fanControlStates[fanType] = !fanControlStates[fanType];
  const button = document.getElementById(fanType + 'FanControlToggle');
  
  if (fanControlStates[fanType]) {
    button.textContent = 'Manual';
    button.classList.remove('active');
    button.style.backgroundColor = '#d35400';
  } else {
    button.textContent = 'Auto';
    button.classList.add('active');
    button.style.backgroundColor = '#e67e22';
  }

  button.disabled = false;
  button.style.pointerEvents = 'auto';

  fetch('/setFanControlMode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: `fan=${fanType}&mode=${fanControlStates[fanType] ? 'manual' : 'auto'}`
  })
  .then(response => response.text())
  .then(data => console.log(data))
  .catch(error => {
    console.error('Error:', error);
    fanControlStates[fanType] = !fanControlStates[fanType];
    updateButtonState(fanType);
  });
}

function updateButtonState(fanType) {
  const button = document.getElementById(fanType + 'FanControlToggle');
  if (fanControlStates[fanType]) {
    button.textContent = 'Manual';
    button.classList.remove('active');
    button.style.backgroundColor = '#d35400';
  } else {
    button.textContent = 'Auto';
    button.classList.add('active');
    button.style.backgroundColor = '#e67e22';
  }
  button.disabled = false;
  button.style.pointerEvents = 'auto';
}

function formatUptime(seconds) {
  let days = Math.floor(seconds / (24 * 3600));
  seconds %= (24 * 3600);
  let hours = Math.floor(seconds / 3600);
  seconds %= 3600;
  let minutes = Math.floor(seconds / 60);
  return `${days}:${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}

function setTemp(value, button) {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/settemp", true);
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("temp=" + value);
  console.log("Sent setTemp:", value);

  pendingSetTemp = value;
  updateTempDisplay(pendingSetTemp);
  var tempSlider = document.getElementById("tempSlider");
  if (tempSlider) tempSlider.value = value;

  if (button) {
    highlightQuickSetButton(value, button);
    lastClickedButton = button;
  }

  var messageDiv = document.getElementById("message");
  if (messageDiv.style.display === "inline-block" && messageDiv.textContent === "Heater was shut down.") {
    messageDiv.textContent = "";
    messageDiv.style.display = "none";
    isShuttingDown = false;
    console.log("Shutdown message cleared by setTemp");
  }
}

function updateSliderAndDisplay(data) {
  var tempSlider = document.getElementById("tempSlider");
  if (tempSlider && !isSliderActive) {
    tempSlider.value = data.setTemp.toFixed(0);
    if (!tempAdjusting || pendingSetTemp === null) {
      updateTempDisplay(data.setTemp.toFixed(0));
    }
  }
}

function highlightQuickSetButton(setTemp, clickedButton = null) {
  const buttons = document.querySelectorAll("#currentSetTemp + div button");
  const roundedSetTemp = Math.round(setTemp);
  buttons.forEach(button => {
    const buttonTemp = QUICK_SET_TEMPS[button.textContent];
    if (clickedButton && button === clickedButton && tempAdjusting) {
      button.style.backgroundColor = "#e67e22";
    } else {
      button.style.backgroundColor = (roundedSetTemp === buttonTemp) ? "#e67e22" : "#d35400";
    }
  });
}

function updateTempDisplay(value) {
  var display = document.getElementById("setTemperatureDisplay");
  if (display) display.textContent = value;
}

function setWallTempTrigger() {
  var triggerInput = document.getElementById("wallTempTriggerInput");
  if (triggerInput) {
    var triggerValue = triggerInput.value;
    if (!isNaN(triggerValue) && triggerValue >= -1 && triggerValue <= 10) {
      var xhr = new XMLHttpRequest();
      xhr.open("POST", "/setWallTempTrigger");
      xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
      xhr.send("trigger=" + triggerValue);
      document.getElementById("currentWallTempTrigger").textContent = triggerValue;
    }
  }
}

function toggleFrostMode(enabled) {
  var label = document.getElementById("frostModeLabel");
  var newState = enabled;
  console.log("Toggling frost mode. New state: " + newState);

  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/frostMode", true);
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("enable=" + newState);

  xhr.onload = function() {
    if (xhr.status === 200) {
      console.log("Frost Mode toggle request successful");
      label.textContent = newState ? "Frost Mode On" : "Frost Mode Off";
      label.classList.toggle("active", newState);
      document.getElementById("frostModeEnable").checked = newState;

      var messageDiv = document.getElementById("message");
      if (messageDiv.style.display === "inline-block" && messageDiv.textContent === "Heater was shut down.") {
        messageDiv.textContent = "";
        messageDiv.style.display = "none";
        isShuttingDown = false;
        console.log("Shutdown message cleared by frost mode toggle");
      }
    } else {
      console.error("Failed to toggle frost mode. Status: " + xhr.status);
      document.getElementById("frostModeEnable").checked = !newState;
    }
  };

  xhr.onerror = function() {
    console.error("Network error while toggling frost mode");
    document.getElementById("frostModeEnable").checked = !newState;
  };
}

function toggleThermostat(enabled) {
  var label = document.getElementById("thermostatLabel");
  var newState = enabled;
  console.log("Toggling thermostat. New state: " + newState);

  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/toggleThermostat");
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("enable=" + newState);

  xhr.onload = function() {
    if (xhr.status === 200) {
      console.log("Thermostat toggle request successful");
      label.textContent = newState ? "Thermostat On" : "Thermostat Off";
      label.classList.toggle("active", newState);
      document.getElementById("thermostatEnable").checked = newState;

      var messageDiv = document.getElementById("message");
      if (messageDiv.style.display === "inline-block" && messageDiv.textContent === "Heater was shut down.") {
        messageDiv.textContent = "";
        messageDiv.style.display = "none";
        isShuttingDown = false;
        console.log("Shutdown message cleared by thermostat toggle");
      }
    } else {
      console.error("Failed to toggle thermostat. Status: " + xhr.status);
      document.getElementById("thermostatEnable").checked = !newState;
    }
  };

  xhr.onerror = function() {
    console.error("Network error while toggling thermostat");
    document.getElementById("thermostatEnable").checked = !newState;
  };
}

function primePump(action) {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/primepump", true);
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("action=" + action);
}

function setTankSize() {
  var tankSizeInput = document.getElementById("tankSizeInput");
  if (tankSizeInput) {
    var tankSize = tankSizeInput.value;
    if (!isNaN(tankSize) && tankSize > 0) {
      var xhr = new XMLHttpRequest();
      xhr.open("POST", "/setTankSize");
      xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
      xhr.send("size=" + tankSize);
      var currentTankSize = document.getElementById("currentTankSize");
      if (currentTankSize) currentTankSize.textContent = tankSize;
    }
  }
}

function resetTank() {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/resetTank");
  xhr.send();
}

function setBLEName() {
  const newName = document.getElementById('bleNameInput').value.trim();
  const validPattern = /^[a-zA-Z0-9\-_]*$/;
  
  if (!validPattern.test(newName)) {
    alert('Name must contain only alphanumeric characters, "-", or "_".');
    return;
  }
  if (newName.length === 0 || newName.length > 32) {
    alert('Name must be between 1 and 32 characters.');
    return;
  }

  fetch('/setName', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: 'name=' + encodeURIComponent(newName)
  })
  .then(response => {
    if (!response.ok) throw new Error('Failed to set BLE name: ' + response.statusText);
    return response.text();
  })
  .then(text => {
    console.log(text);
    document.getElementById('currentBLEName').textContent = newName;
    document.getElementById('bleNameInput').value = '';
    setTimeout(() => {
      window.location.href = 'http://' + newName + '.local';
    }, 15000);
    isEditingBLEName = false;
  })
  .catch(error => {
    console.error('Error setting BLE name:', error);
    alert('Failed to set BLE name. Check console for details.');
    isEditingBLEName = false;
  });
}

function savePreferences() {
  const filename = document.getElementById('backupFilename').value.trim();
  if (!filename) {
    showPrefsMessage('Please enter a filename.', 'red');
    return;
  }

  fetch('/managePreferences', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/x-www-form-urlencoded',
    },
    body: new URLSearchParams({
      action: 'dump',
      filename: "/" + filename
    })
  })
  .then(response => {
    if (!response.ok) {
      throw new Error('Failed to save preferences: ' + response.statusText);
    }
    return response.text();
  })
  .then(text => {
    showPrefsMessage(text, '#f39c12');
    setTimeout(() => clearPrefsMessage(), 5000); // Clear message after 5 seconds
  })
  .catch(error => {
    showPrefsMessage(error.message, 'red');
  });
}

function loadPreferences() {
  const filename = document.getElementById('backupFilename').value.trim();
  if (!filename) {
    showPrefsMessage('Please enter a filename.', 'red');
    return;
  }

  fetch('/managePreferences', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/x-www-form-urlencoded',
    },
    body: new URLSearchParams({
      action: 'load',
      filename: "/" + filename
    })
  })
  .then(response => {
    if (!response.ok) {
      throw new Error('Failed to load preferences: ' + response.statusText);
    }
    return response.text();
  })
  .then(text => {
    showPrefsMessage(text, '#f39c12');
    // Optionally refresh UI elements that depend on preferences
    updateUIAfterLoad();
    setTimeout(() => clearPrefsMessage(), 5000); // Clear message after 5 seconds
  })
  .catch(error => {
    showPrefsMessage(error.message, 'red');
  });
}

function showPrefsMessage(message, color) {
  const messageElement = document.getElementById('prefsMessage');
  messageElement.textContent = message;
  messageElement.style.color = color;
}

function clearPrefsMessage() {
  const messageElement = document.getElementById('prefsMessage');
  messageElement.textContent = '';
}

function updateUIAfterLoad() {
  // Update UI elements that depend on loaded preferences
  fetch('/getStatus') // Assuming you have an endpoint to get current status
    .then(response => response.json())
    .then(data => {
      // Update BLE name
      document.getElementById('currentBLEName').textContent = data.bleName || 'HEATER-THERM';
      // Update ZIP code
      document.getElementById('currentZipCode').textContent = data.zipcode || '64856';
      // Update thermostat and fan settings if applicable
      document.getElementById('thermostatEnable').checked = data.thermostatMode || false;
      document.getElementById('thermostatLabel').textContent = data.thermostatMode ? 'Thermostat On' : 'Thermostat Off';
      document.getElementById('frostModeEnable').checked = data.frostMode || false;
      document.getElementById('frostModeLabel').textContent = data.frostMode ? 'Frost Mode On' : 'Frost Mode';
      // Add more updates as needed based on your preferences
    })
    .catch(error => console.error('Error updating UI:', error));
}

function shutdownHeater() {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/shutdownHeater");
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.onload = function() {
    if (xhr.status === 200) {
      console.log("Shut down command sent.");
      var messageDiv = document.getElementById("message");
      messageDiv.textContent = "Shut down command sent.";
      messageDiv.style.display = "inline-block";
      isShuttingDown = true;
    }
  };
  xhr.send();
}

function turnHeaterOn() {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/turnHeaterOn");
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.onload = function() {
    if (xhr.status === 200) {
      console.log("Heater on command sent.");
      var messageDiv = document.getElementById("message");
      messageDiv.textContent = "Heater on command sent.";
      messageDiv.style.display = "inline-block";
      isShuttingDown = false;
      controlEnable = 1;
    } else {
      console.error("Failed to turn heater on. Status: " + xhr.status);
    }
  };
  xhr.onerror = function() {
    console.error("Network error while turning heater on");
  };
  xhr.send();
}

function filterDuplicates(values, timestamps) {
  if (values.length <= 1) return { values, timestamps };
  const filtered = [];
  let i = 0;
  while (i < values.length) {
    let startIdx = i;
    while (i + 1 < values.length && values[i + 1] === values[startIdx]) {
      i++;
    }
    filtered.push({ value: values[startIdx], timestamp: timestamps[startIdx] });
    if (i > startIdx) {
      filtered.push({ value: values[i], timestamp: timestamps[i] });
    }
    i++;
  }
  if (filtered.length === 0 || filtered[filtered.length - 1].timestamp !== timestamps[timestamps.length - 1]) {
    filtered.push({ value: values[values.length - 1], timestamp: timestamps[timestamps.length - 1] });
  }
  return {
    values: filtered.map(item => item.value),
    timestamps: filtered.map(item => item.timestamp)
  };
}

function updateThermState(serialActive) {
  var thermDiv = document.getElementById("therm");
  if (thermDiv) {
    if (!serialActive) {
      thermDiv.style.opacity = "0.5";
      thermDiv.style.pointerEvents = "none";
      thermDiv.style.filter = "grayscale(100%)";
    } else {
      thermDiv.style.opacity = "1";
      thermDiv.style.pointerEvents = "auto";
      thermDiv.style.filter = "none";
    }
  }
}

function deleteFile(filename) {
  if (confirm(`Are you sure you want to delete ${filename}?`)) {
    fetch('/deleteFile', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `filename=${encodeURIComponent(filename)}`
    })
      .then(response => response.text())
      .then(result => {
        alert(result);
        listfiles();
      })
      .catch(error => console.error('Error deleting file:', error));
  }
}

document.addEventListener('DOMContentLoaded', function() {
  var tempSlider = document.getElementById("tempSlider");
  var currentSetTemp = document.getElementById("currentSetTemp");
  let lastClickedButton = null;
  let serverZipCode = "64856";
  let hasGeolocationFailed = false;

  const outdoorDiv = document.getElementById('outdoor');
  if (outdoorDiv) {
    outdoorDiv.style.cursor = 'pointer';
    outdoorDiv.addEventListener('click', function() {
      const weatherUrl = `https://www.bing.com/search?q=weather+${serverZipCode}`;
      console.log('Opening Bing Weather URL:', weatherUrl);
      window.open(weatherUrl, '_blank');
    });
  }

  const bleNameInput = document.getElementById('bleNameInput');
  if (bleNameInput) {
    bleNameInput.addEventListener('focus', function() { isEditingBLEName = true; console.log("Started editing BLE name"); });
    bleNameInput.addEventListener('blur', function() {
      setTimeout(() => {
        if (bleNameInput.value.trim() === '') {
          isEditingBLEName = false;
          console.log("Stopped editing BLE name (blur with empty input)");
        }
      }, 100);
    });
    bleNameInput.addEventListener('input', function() {
      const validPattern = /^[a-zA-Z0-9\-_]*$/;
      if (!validPattern.test(bleNameInput.value)) bleNameInput.value = bleNameInput.value.replace(/[^a-zA-Z0-9\-_]/g, '');
      if (bleNameInput.value.length > 32) bleNameInput.value = bleNameInput.value.substring(0, 32);
    });
    bleNameInput.addEventListener('keypress', function(e) { if (e.key === 'Enter') setBLEName(); });
  }

  document.getElementById("setBLENameButton")?.addEventListener('click', setBLEName);

  function fetchZipCodeFromCoords(lat, lon) {
    console.log(`Fetching zip code for lat: ${lat}, lon: ${lon}`);
    const apiKey = "aeb9ccaba969c927fc2b8ce501da53a8";
    const url = `http://api.openweathermap.org/geo/1.0/reverse?lat=${lat}&lon=${lon}&limit=1&appid=${apiKey}`;
    return fetch(url)
      .then(response => {
        if (!response.ok) throw new Error(`HTTP error! Status: ${response.status}`);
        return response.json();
      })
      .then(data => {
        console.log("Geocoding API response:", data);
        if (data && data.length > 0 && data[0].zip) {
          const zipCode = data[0].zip.split(',')[0];
          if (/^\d{5}$/.test(zipCode)) {
            console.log(`Fetched zip code: ${zipCode}`);
            return zipCode;
          } else {
            throw new Error("Invalid zip code format: " + zipCode);
          }
        } else {
          throw new Error("No zip code found for location");
        }
      });
  }

  function setZipCode(zipCode) {
    return new Promise((resolve, reject) => {
      console.log(`Sending zip code to server: ${zipCode}`);
      var xhr = new XMLHttpRequest();
      xhr.open("POST", "/setZipCode", true);
      xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
      xhr.onload = function() {
        if (xhr.status === 200) {
          document.getElementById("currentZipCode").textContent = zipCode;
          console.log("Zip code successfully updated to: " + zipCode);
          resolve();
        } else {
          console.error("Failed to update zip code: " + xhr.responseText);
          reject(new Error("Failed to update zip code: " + xhr.responseText));
        }
      };
      xhr.onerror = function() {
        console.error("Network error while updating zip code");
        reject(new Error("Network error while updating zip code"));
      };
      xhr.send("zipcode=" + zipCode);
    });
  }

  function manualSetZipCode() {
    var zipInput = document.getElementById("zipCodeInput");
    if (zipInput) {
      var zipCode = zipInput.value.trim();
      if (/^\d{5}$/.test(zipCode)) {
        setZipCode(zipCode)
          .then(() => {
            document.getElementById("manualZipInput").style.display = "none";
            document.getElementById("zipErrorMessage").textContent = "";
            hasGeolocationFailed = false;
            console.log("Manual zip code set successfully");
          })
          .catch(error => {
            document.getElementById("zipErrorMessage").textContent = error.message;
            console.error("Manual zip set failed:", error);
          });
      } else {
        document.getElementById("zipErrorMessage").textContent = "Please enter a valid 5-digit ZIP code.";
        console.warn("Invalid manual zip code input:", zipCode);
      }
    }
  }

  function checkAndUpdateZipCode() {
    console.log("Starting geolocation check...");
    if (!navigator.geolocation) {
      console.error("Geolocation not supported by browser");
      showManualInput("Geolocation is not supported by your browser.");
      return;
    }

    navigator.geolocation.getCurrentPosition(
      function(position) {
        console.log("Geolocation success:", position.coords);
        const lat = position.coords.latitude;
        const lon = position.coords.longitude;
        fetchZipCodeFromCoords(lat, lon)
          .then(zipCode => {
            console.log(`Client zip: ${zipCode}, Server zip: ${serverZipCode}`);
            if (zipCode !== serverZipCode) {
              console.log("Zip codes differ, updating server...");
              setZipCode(zipCode)
                .catch(error => {
                  console.error("Failed to set zip code:", error);
                  showManualInput("Failed to update zip code: " + error.message);
                });
            } else {
              console.log("Zip codes match, no update needed.");
            }
          })
          .catch(error => {
            console.error("Error fetching zip code:", error);
            showManualInput("Failed to fetch zip code: " + error.message);
          });
      },
      function(error) {
        let errorMsg = "Geolocation error: ";
        switch(error.code) {
          case error.PERMISSION_DENIED: errorMsg += "User denied the request for Geolocation."; break;
          case error.POSITION_UNAVAILABLE: errorMsg += "Location information is unavailable."; break;
          case error.TIMEOUT: errorMsg += "The request to get user location timed out."; break;
          default: errorMsg += "An unknown error occurred.";
        }
        console.error(errorMsg);
        showManualInput(errorMsg);
      },
      { timeout: 10000 }
    );
  }

  function showManualInput(errorMessage) {
    if (!hasGeolocationFailed) {
      hasGeolocationFailed = true;
      console.log("Showing manual input due to:", errorMessage);
      document.getElementById("zipErrorMessage").textContent = errorMessage;
      document.getElementById("manualZipInput").style.display = "block";
    }
  }

  document.getElementById("setZipButton").addEventListener("click", manualSetZipCode);

  drawGauge('ductFanGauge', 0);
  drawGauge('wallFanGauge', 0);

  document.getElementById('ductFanGauge').addEventListener('click', (e) => handleGaugeInteraction('duct', e));
  document.getElementById('wallFanGauge').addEventListener('click', (e) => handleGaugeInteraction('wall', e));

  const ductToggleButton = document.getElementById('ductFanControlToggle');
  const wallToggleButton = document.getElementById('wallFanControlToggle');
  if (ductToggleButton) updateButtonState('duct');
  else console.error('ductFanControlToggle button not found in DOM');
  if (wallToggleButton) updateButtonState('wall');
  else console.error('wallFanControlToggle button not found in DOM');

  const customLocale = {
    localize: {
      month: ['Jan', 'Feb', 'Mar', 'Apr', 'May', 'Jun', 'Jul', 'Aug', 'Sep', 'Oct', 'Nov', 'Dec'],
      day: ['Sun', 'Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat']
    },
    formatLong: {
      date: () => 'MM/dd/yyyy',
      time: () => 'HH:mm:ss',
      dateTime: () => 'MM/dd/yyyy HH:mm:ss'
    }
  };

  // Initialize charts
  console.log("Initializing charts...");
  if (typeof Chart === 'undefined') {
    console.error("Chart.js is not loaded!");
  } else {
    tempChart = new Chart(document.getElementById('tempChart').getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'Indoor Temp (°F)', data: [], fill: false, borderColor: 'orange', tension: 0.1, yAxisID: 'y', spanGaps: true },
          { label: 'Outdoor Temp (°F)', data: [], fill: false, borderColor: 'blue', tension: 0.1, yAxisID: 'y', spanGaps: true },
          { label: 'Pump Hz', data: [], fill: false, borderColor: 'green', tension: 0.1, yAxisID: 'y1', spanGaps: true }
        ]
      },
      options: {
        animation: false,
        scales: {
          x: { type: 'time', time: { unit: 'minute', displayFormats: { minute: 'H:mm' }, tooltipFormat: 'HH:mm' }, adapters: { date: { locale: customLocale } }, title: { display: false } },
          y: {
            title: { display: true, text: 'Temperature (°F)', color: 'white' }, position: 'left',
            ticks: { stepSize: 2, callback: value => value + '°F', color: 'grey' },
            afterDataLimits: scale => {
              const allTemps = [...scale.chart.data.datasets[0].data, ...scale.chart.data.datasets[1].data].filter(v => v !== null && !isNaN(v));
              if (allTemps.length === 0) return;
              const dataMin = Math.min(...allTemps);
              const dataMax = Math.max(...allTemps);
              const midpoint = (dataMin + dataMax) / 2;
              const range = dataMax - dataMin;
              let min = range < 12 ? midpoint - 7 : dataMin - 1;
              let max = range < 12 ? midpoint + 7 : dataMax + 1;
              min = Math.floor(min / 2) * 2;
              max = Math.ceil(max / 2) * 2;
              scale.options.min = min;
              scale.options.max = max;
            },
            grid: { color: 'rgba(255, 255, 255, 0.1)' }
          },
          y1: {
            title: { display: true, text: 'Pump Frequency (Hz)', color: 'white' }, position: 'right', min: -1, max: 7,
            ticks: { stepSize: 1, callback: value => value + ' Hz', color: 'grey' }, grid: { drawOnChartArea: false }
          }
        },
        plugins: { legend: { labels: { color: 'grey' } } }
      },
      plugins: [{
        beforeDraw: chart => {
          const ctx = chart.canvas.getContext('2d');
          ctx.save();
          ctx.globalCompositeOperation = 'destination-over';
          ctx.fillStyle = 'lightyellow';
          ctx.fillRect(0, 0, chart.width, chart.height);
          ctx.restore();
        }
      }]
    });
    console.log("tempChart initialized:", !!tempChart);

    voltageChart = new Chart(document.getElementById('voltageChart').getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'Voltage (V)', data: [], fill: false, borderColor: 'red', tension: 0.1, spanGaps: true, yAxisID: 'y-voltage' },
          { label: 'Current (A)', data: [], fill: false, borderColor: 'blue', tension: 0.1, spanGaps: true, yAxisID: 'y-amps' }
        ]
      },
      options: {
        animation: false,
        scales: {
          x: { type: 'time', time: { unit: 'minute', displayFormats: { minute: 'H:mm' }, tooltipFormat: 'HH:mm' }, adapters: { date: { locale: customLocale } }, title: { display: false } },
          'y-voltage': {
            title: { display: false }, position: 'left',
            ticks: { stepSize: 0.5, callback: value => value + 'V', color: 'grey' },
            afterDataLimits: scale => {
              const voltages = scale.chart.data.datasets[0].data.filter(v => v !== null && !isNaN(v));
              if (voltages.length === 0) return;
              const dataMin = Math.min(...voltages);
              const dataMax = Math.max(...voltages);
              const midpoint = (dataMin + dataMax) / 2;
              const range = dataMax - dataMin;
              let min = range < 3 ? midpoint - 1.5 : dataMin - 0.25;
              let max = range < 3 ? midpoint + 1.5 : dataMax + 0.25;
              min = Math.round(min / 0.5) * 0.5;
              max = Math.round(max / 0.5) * 0.5;
              if (dataMin >= 10) min = 10;
              if (max < 15) max = 15;
              scale.options.min = min;
              scale.options.max = max;
            },
            grid: { color: 'rgba(255, 255, 255, 0.1)' }
          },
          'y-amps': {
            title: { display: false }, position: 'right',
            ticks: { stepSize: 1, callback: value => value + 'A', color: 'grey' },
            afterDataLimits: scale => {
              const amps = scale.chart.data.datasets[1].data.filter(v => v !== null && !isNaN(v));
              if (amps.length === 0) { scale.options.min = 0; scale.options.max = 5; return; }
              const dataMin = Math.min(...amps);
              const dataMax = Math.max(...amps);
              const range = dataMax - dataMin;
              let min = 0;
              let max = range < 3 ? dataMax + 1.5 : dataMax + 0.5;
              max = Math.ceil(max / 1) * 1;
              if (max < 5) max = 5;
              scale.options.min = min;
              scale.options.max = max;
            },
            grid: { drawOnChartArea: false }
          }
        },
        plugins: { legend: { labels: { color: 'grey' } } }
      },
      plugins: [{
        beforeDraw: chart => {
          const ctx = chart.canvas.getContext('2d');
          ctx.save();
          ctx.globalCompositeOperation = 'destination-over';
          ctx.fillStyle = 'lightyellow';
          ctx.fillRect(0, 0, chart.width, chart.height);
          ctx.restore();
        }
      }]
    });
    console.log("voltageChart initialized:", !!voltageChart);

    hourlyFuelChart = new Chart(document.getElementById('hourlyFuelChart').getContext('2d'), {
      type: 'bar',
      data: {
        labels: [],
        datasets: [
          { label: 'Gal Per HR', data: [], backgroundColor: 'rgba(255, 165, 0, 0.7)', borderColor: 'rgba(255, 165, 0, 1)', borderWidth: 1 },
          { label: 'Current Hour (In Progress)', data: [], backgroundColor: 'rgba(255, 165, 0, 0.3)', borderColor: 'rgba(255, 165, 0, 1)', borderWidth: 1 }
        ]
      },
      options: {
        animation: false,
        scales: {
          x: { type: 'time', time: { unit: 'hour', displayFormats: { hour: 'H' }, tooltipFormat: 'MM/dd H:00' }, adapters: { date: { locale: customLocale } }, title: { display: false } },
          y: {
            beginAtZero: true, title: { display: false },
            ticks: { stepSize: 0.005, callback: value => value.toFixed(2) + ' gal', color: 'grey' },
            afterDataLimits: scale => {
              const allData = [...scale.chart.data.datasets[0].data, ...scale.chart.data.datasets[1].data].filter(v => v !== null && !isNaN(v));
              if (allData.length === 0) return;
              const max = Math.max(...allData);
              scale.options.max = Math.ceil(max * 1.1 / 0.05) * 0.05;
            }
          }
        },
        plugins: { legend: { labels: { color: 'grey' } } }
      },
      plugins: [{
        beforeDraw: chart => {
          const ctx = chart.canvas.getContext('2d');
          ctx.save();
          ctx.globalCompositeOperation = 'destination-over';
          ctx.fillStyle = 'lightyellow';
          ctx.fillRect(0, 0, chart.width, chart.height);
          ctx.restore();
        }
      }]
    });
    console.log("hourlyFuelChart initialized:", !!hourlyFuelChart);

    wattHourChart = new Chart(document.getElementById('wattHourChart').getContext('2d'), {
      type: 'bar',
      data: {
        labels: [],
        datasets: [
          { label: 'Watt', data: [], backgroundColor: 'rgba(0, 128, 255, 0.7)', borderColor: 'rgba(0, 128, 255, 1)', borderWidth: 1 },
          { label: 'Watt (In Progress)', data: [], backgroundColor: 'rgba(0, 128, 255, 0.3)', borderColor: 'rgba(0, 128, 255, 1)', borderWidth: 1 }
        ]
      },
      options: {
        animation: false,
        scales: {
          x: { type: 'time', time: { unit: 'hour', displayFormats: { hour: 'H' }, tooltipFormat: 'MM/dd H:00' }, adapters: { date: { locale: customLocale } }, title: { display: false } },
          y: {
            beginAtZero: true, title: { display: false },
            ticks: { stepSize: 10, callback: value => value.toFixed(0) + ' W', color: 'grey' },
            afterDataLimits: scale => {
              const allData = [...scale.chart.data.datasets[0].data, ...scale.chart.data.datasets[1].data].filter(v => v !== null && !isNaN(v));
              if (allData.length === 0) return;
              const max = Math.max(...allData);
              scale.options.max = Math.ceil(max * 1.1 / 10) * 10;
            }
          }
        },
        plugins: { legend: { labels: { color: 'grey' } } }
      },
      plugins: [{
        beforeDraw: chart => {
          const ctx = chart.canvas.getContext('2d');
          ctx.save();
          ctx.globalCompositeOperation = 'destination-over';
          ctx.fillStyle = 'lightyellow';
          ctx.fillRect(0, 0, chart.width, chart.height);
          ctx.restore();
        }
      }]
    });
    console.log("wattHourChart initialized:", !!wattHourChart);
  }

  if (tempSlider) {
    tempSlider.addEventListener('mousedown', function() { isSliderActive = true; console.log("Slider adjustment started"); });
    tempSlider.addEventListener('touchstart', function() { isSliderActive = true; console.log("Slider adjustment started (touch)"); });
    tempSlider.oninput = function() { updateTempDisplay(this.value); };
    tempSlider.addEventListener('mouseup', function() {
      isSliderActive = false;
      pendingSetTemp = this.value;
      setTemp(pendingSetTemp, null);
      updateTempDisplay(pendingSetTemp);
      console.log("Slider adjustment ended");
    });
    tempSlider.addEventListener('touchend', function() {
      isSliderActive = false;
      pendingSetTemp = this.value;
      setTemp(pendingSetTemp, null);
      updateTempDisplay(pendingSetTemp);
      console.log("Slider adjustment ended (touch)");
    });
  }

  var listFilesBtn = document.getElementById('listFilesBtn');
  if (listFilesBtn) listFilesBtn.addEventListener('click', listfiles);

  function listfiles() {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/listfiles", true);
    xhr.onload = function() {
      if (xhr.status === 200) {
        var fileList = JSON.parse(xhr.responseText);
        updateFileList(fileList.files);
      } else {
        console.error("Failed to list files, status:", xhr.status);
      }
    };
    xhr.onerror = function() { console.error("Network error listing files"); };
    xhr.send();
  }

function updateFileList(files) {
    var tableBody = document.getElementById('fileListBody');
    if (tableBody) {
      tableBody.innerHTML = '';
      files.forEach(function(file) {
        var row = tableBody.insertRow();
        // Create a clickable link using file.name
        row.insertCell(0).innerHTML = `<a href="/${encodeURIComponent(file.name)}" class="file-link" target="_blank">${file.name}</a>`;
        row.insertCell(1).textContent = file.size;
        row.insertCell(2).innerHTML = `<div id="delete"><button class="delete-btn" onclick="deleteFile('${file.name}')">Delete</button></div>`;
      });
    }
  }

  // Setup EventSource
  var evtSource = new EventSource("/events");

  evtSource.onmessage = function(e) {
    console.log("Event received:", e.data);
    if (e.data && typeof e.data === 'string' && e.data.trim() !== '') {
      try {
        var eventLines = e.data.split('\n');
        if (eventLines.length < 2) {
          console.error("Received malformed SSE event:", e.data);
          return;
        }

        var eventNameLine = eventLines[0].split(': ');
        var eventDataLine = eventLines[1].split(': ');
        if (eventNameLine[0] !== 'event' || !eventNameLine[1] || eventDataLine[0] !== 'data' || !eventDataLine[1]) {
          console.error("Invalid event format:", e.data);
          return;
        }

        var eventName = eventNameLine[1];
        var eventData = eventDataLine[1];
        var data = JSON.parse(eventData);

        switch (eventName) {
          case 'heater_update':
            if (data.bleName && !isEditingBLEName) document.getElementById('currentBLEName').textContent = data.bleName;
            if (data.zipcode) {
              serverZipCode = data.zipcode;
              document.getElementById("currentZipCode").textContent = serverZipCode;
            }

            updateSliderAndDisplay(data);

            tempAdjusting = data.tempadjusting || false;
            if (tempAdjusting) {
              currentSetTemp.classList.add('adjusting');
              updateTempDisplay(data.targettemp !== undefined ? data.targettemp : (pendingSetTemp !== null ? pendingSetTemp : data.setTemp.toFixed(0)));
              if (data.targettemp !== undefined) {
                highlightQuickSetButton(data.targettemp);
              } else if (pendingSetTemp !== null && lastClickedButton) {
                highlightQuickSetButton(pendingSetTemp, lastClickedButton);
              } else {
                highlightQuickSetButton(data.setTemp);
              }
            } else {
              currentSetTemp.classList.remove('adjusting');
              pendingSetTemp = null;
              lastClickedButton = null;
              if (!isSliderActive) {
                updateTempDisplay(data.setTemp.toFixed(0));
                highlightQuickSetButton(data.setTemp);
              }
            }

            const utcDateTime = new Date(data.epochTime * 1000);
            const timeOptions = { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: true };
            const dateOptions = { year: 'numeric', month: '2-digit', day: '2-digit' };
            document.getElementById("currentTime").querySelector("span").textContent = utcDateTime.toLocaleTimeString([], timeOptions);
            document.getElementById("currentDate").querySelector("span").textContent = utcDateTime.toLocaleDateString([], dateOptions);

            document.getElementById("currentTemp").textContent = data.currentTemp + "°F";
            document.getElementById("heaterState").textContent = data.state;
            document.getElementById("runtime").textContent = data.heaterHourMeter.toFixed(2) + "Hrs";
            document.getElementById("currentWallTempTrigger").textContent = data.walltemptrigger.toFixed(0);
            document.getElementById("uptime").querySelector("span").textContent = formatUptime(parseInt(data.uptime));
            document.getElementById("lifetimeFuel").textContent = data.fuelConsumedLifetime.toFixed(2) + " Gal";
            document.getElementById("tankFuel").textContent = data.fuelConsumedTank.toFixed(3) + " Gal";
            document.getElementById("currentUsage").textContent = data.currentUsage.toFixed(3) + " GPH";
            document.getElementById("currentTankSize").textContent = data.tankSizeGallons.toFixed(0) + "Gal";
            document.getElementById("avgGPH").textContent = data.averageGPH.toFixed(2) + "GPH";
            document.getElementById("fanSpeed").textContent = data.fanSpeed + "RPM";
            document.getElementById("supplyVoltage").textContent = data.supplyVoltage.toFixed(1) + "V";
            document.getElementById("voltageWarning").textContent = data.voltageWarning;
            document.getElementById("glowPlugHours").textContent = data.glowPlugHours.toFixed(2) + "Hrs";
            document.getElementById("rollingAvgGPH").textContent = data.rollingAvgGPH.toFixed(2) + " GPH";
            document.getElementById("rollingRuntimeHours").textContent = (data.rollingRuntimeHours === null ? '∞' : data.rollingRuntimeHours.toFixed(2)) + " Hrs";
            document.getElementById("tankruntime").textContent = (data.remainingRuntimeHours == null ? '∞' : data.remainingRuntimeHours.toFixed(2)) + "Hrs";
            document.getElementById("avgWattHours24h").textContent = data.avgWattHours24h.toFixed(2) + " W/Hr";
            const tempWarnElement = document.getElementById("tempWarn");
            if (tempWarnElement) {
              switch (data.tempwarn) {
                case 3: tempWarnElement.textContent = "HEATER INTERNAL TEMP OUT OF SPEC!"; tempWarnElement.style.display = "inline-block"; tempWarnElement.style.color = "red"; break;
                case 2: tempWarnElement.textContent = "Shut down. Wall Temp > 120F!"; tempWarnElement.style.display = "inline-block"; tempWarnElement.style.color = "red"; break;
                case 1: tempWarnElement.textContent = "Wall temp > 110F."; tempWarnElement.style.display = "inline-block"; tempWarnElement.style.color = "orange"; break;
                case 0: default: tempWarnElement.textContent = ""; tempWarnElement.style.display = "none"; break;
              }
            }

            const heaterErrorElement = document.getElementById("heatererror");
            const heaterErrorTxtElement = document.getElementById("heatererrortxt");
            if (heaterErrorElement && heaterErrorTxtElement) {
              heaterErrorTxtElement.textContent = (data.errornum > 1) ? (data.error || "Unknown Error") : "";
              heaterErrorElement.style.display = (data.errornum > 1 || data.tempwarn > 0) ? "inline-block" : "none";
            }

            const supplyVoltage = (data.supplyVoltage <= 5.0 || data.supplyVoltage > 15.0 || isNaN(data.supplyVoltage)) ? 12.0 : data.supplyVoltage;

            fanControlStates.duct = data.ductFanManualControl;
            updateButtonState('duct');
            if (fanControlStates.duct) {
              const ductPwm = data.manualDuctFanSpeed;
              let ductVoltage = data.manualDuctFanVoltage;
              let ductPercent = ductPwm === 0 ? 0 : ((ductVoltage - 10.5) / (supplyVoltage - 10.5)) * 95 + 5;
              ductPercent = Math.round(ductPercent / 5) * 5;
              ductPercent = Math.max(0, Math.min(100, ductPercent));
              if (ductVoltage === 0 && ductPwm > 0) ductVoltage = (ductPwm / 1023) * supplyVoltage;
              drawGauge('ductFanGauge', ductPercent);
              document.getElementById('ductFanGaugeValue').textContent = ductPercent + '%';
              document.getElementById('ductfan').textContent = ductPwm === 0 ? "Man Off" : `${ductVoltage.toFixed(1)}V (${ductPwm})`;
            } else {
              const ductPwm = data.ductfan;
              let ductVoltage = ductPwm === 0 ? 0.0 : 10.5 + (ductPwm / 1023) * (supplyVoltage - 10.5);
              ductVoltage = Math.min(supplyVoltage, Math.max(10.5, ductVoltage));
              drawGauge('ductFanGauge', 0);
              document.getElementById('ductFanGaugeValue').textContent = '0%';
              document.getElementById('ductfan').textContent = ductPwm === 0 ? "Off" : `${ductVoltage.toFixed(1)}V (${ductPwm})`;
            }

            fanControlStates.wall = data.wallFanManualControl;
            updateButtonState('wall');
            if (fanControlStates.wall) {
              const wallPwm = data.manualWallFanSpeed;
              let wallVoltage = data.manualWallFanVoltage;
              let wallPercent = wallPwm === 0 ? 0 : ((wallVoltage - 6) / (supplyVoltage - 6)) * 95 + 5;
              wallPercent = Math.round(wallPercent / 5) * 5;
              wallPercent = Math.max(0, Math.min(100, wallPercent));
              if (wallVoltage === 0 && wallPwm > 0) wallVoltage = (wallPwm / 1023) * supplyVoltage;
              drawGauge('wallFanGauge', wallPercent);
              document.getElementById('wallFanGaugeValue').textContent = wallPercent + '%';
              document.getElementById('wallfan').textContent = wallPwm === 0 ? "Man Off" : `${wallVoltage.toFixed(1)}V (${wallPwm})`;
            } else {
              const wallPwm = data.wallfan;
              let wallVoltage = wallPwm === 0 ? 0.0 : 6 + (wallPwm / 1023) * (7.5 - 6);
              wallVoltage = Math.min(supplyVoltage, Math.max(5, wallVoltage));
              drawGauge('wallFanGauge', 0);
              document.getElementById('wallFanGaugeValue').textContent = '0%';
              document.getElementById('wallfan').textContent = wallPwm === 0 ? "Off" : `${wallVoltage.toFixed(1)}V (${wallPwm})`;
            }

            document.getElementById("thermostatLabel").textContent = data.controlEnable ? "Thermostat On" : "Thermostat Off";
            document.getElementById("thermostatLabel").classList.toggle("active", data.controlEnable);
            document.getElementById("thermostatEnable").checked = data.controlEnable;

            document.getElementById("frostModeLabel").textContent = data.frostMode ? "Frost Mode On" : "Frost Mode Off";
            document.getElementById("frostModeLabel").classList.toggle("active", data.frostMode);
            document.getElementById("frostModeEnable").checked = data.frostMode;

            document.getElementById("heaterInternalTemp").textContent = data.heaterinternalTemp + "°F";
            document.getElementById("glowPlugCurrent").textContent = data.glowPlugCurrent_Amps.toFixed(2);
            document.getElementById("pumpHz").textContent = data.pumpHz;
            document.getElementById("walltemp").textContent = data.walltemp + "°F";

            var tankSizeGallons = data.tankSizeGallons;
            var fuelConsumedTank = data.fuelConsumedTank;
            var gallonsLeft = tankSizeGallons > 0 ? tankSizeGallons - fuelConsumedTank : 0;
            gallonsLeft = Math.max(0, gallonsLeft);
            var fuelPercentage = tankSizeGallons > 0 ? (gallonsLeft / tankSizeGallons) * 100 : 0;
            fuelPercentage = Math.max(0, Math.min(100, fuelPercentage));
            document.getElementById("fuelGallonsLeft").textContent = gallonsLeft.toFixed(2) + " Gal";
            document.getElementById("fuelGaugeFill").style.transform = `rotate(${(fuelPercentage / 100) * 180}deg)`;
            var fuelGaugeFill = document.getElementById("fuelGaugeFill");
            if (fuelPercentage < 10) fuelGaugeFill.style.backgroundColor = "#FF0000";
            else if (fuelPercentage < 25) fuelGaugeFill.style.backgroundColor = "#FFA500";
            else if (fuelPercentage < 50) fuelGaugeFill.style.backgroundColor = "#FFFF";
            else fuelGaugeFill.style.backgroundColor = "#327A24";

            let outdoorValue = data.outsideTempF !== undefined && data.outsideHumidity !== undefined
              ? `${data.outsideTempF.toFixed(0)}°F / ${data.outsideHumidity.toFixed(0)}%`
              : data.outsideTempF !== undefined
              ? `${data.outsideTempF.toFixed(1)}°F / N/A%`
              : data.outsideHumidity !== undefined
              ? `N/A°F / ${data.outsideHumidity.toFixed(1)}%`
              : "N/A";
            document.getElementById('outdoor').innerText = outdoorValue;

            if (data.serialActive !== undefined) updateThermState(data.serialActive);

            var messageDiv = document.getElementById("message");
            if (isShuttingDown && data.statenum === 0) {
              messageDiv.textContent = "Heater was shut down.";
              messageDiv.style.display = "inline-block";
            } else if (!isShuttingDown && messageDiv.textContent === "Heater was shut down.") {
              // Don't clear here
            } else if (!isShuttingDown && tankSizeGallons > 0 && fuelConsumedTank >= tankSizeGallons * 0.90) {
              messageDiv.textContent = "Warning: Fuel level is low!";
              messageDiv.style.display = "inline-block";
            } else if (!isShuttingDown) {
              messageDiv.textContent = "";
              messageDiv.style.display = "none";
            }

            if (messageDiv && data.serialEstablished !== undefined) {
              if (data.serialEstablished === true && data.serialActive === false) {
                messageDiv.textContent = "Heater communication interrupted...";
                messageDiv.style.display = "inline-block";
              } else if (data.serialEstablished === false) {
                messageDiv.textContent = "Establishing Heater Communications...";
                messageDiv.style.display = "inline-block";
              } else if (messageDiv.textContent === "Heater communication interrupted..." || messageDiv.textContent === "Establishing Heater Communications...") {
                messageDiv.textContent = "";
                messageDiv.style.display = "none";
              }
            }
            break;

          case 'temp_history_update':
            console.log("Processing temp_history_update");
            const tempFiltered = filterDuplicates(data.tempHistory, data.timestamps);
            tempHistoryData = { values: tempFiltered.values, timestamps: tempFiltered.timestamps };
            console.log("tempHistoryData updated:", tempHistoryData);

            if (!tempChart) {
              console.error("tempChart not initialized!");
              break;
            }

            const tempTimestamps = [...new Set([
              ...tempHistoryData.timestamps,
              ...outsideTempHistoryData.timestamps,
              ...pumpHzHistoryData.timestamps
            ])].sort((a, b) => a - b);

            tempChart.data.labels = tempTimestamps.map(t => new Date(t * 1000));
            tempChart.data.datasets[0].data = alignData(tempHistoryData.values, tempHistoryData.timestamps, tempTimestamps);
            tempChart.data.datasets[1].data = alignData(outsideTempHistoryData.values, outsideTempHistoryData.timestamps, tempTimestamps);
            tempChart.data.datasets[2].data = alignData(pumpHzHistoryData.values, pumpHzHistoryData.timestamps, tempTimestamps);

            console.log("tempChart labels:", tempChart.data.labels);
            console.log("tempChart datasets:", tempChart.data.datasets);
            tempChart.update('none');
            tempChart.resize();
            console.log("tempChart updated with indoor temp data");
            break;

          case 'outside_temp_history_update':
            console.log("Processing outside_temp_history_update");
            const outsideTempFiltered = filterDuplicates(data.outsideTempHistory, data.timestamps);
            outsideTempHistoryData = { values: outsideTempFiltered.values, timestamps: outsideTempFiltered.timestamps };
            console.log("outsideTempHistoryData updated:", outsideTempHistoryData);

            if (!tempChart) {
              console.error("tempChart not initialized!");
              break;
            }

            const outsideTempTimestamps = [...new Set([
              ...tempHistoryData.timestamps,
              ...outsideTempHistoryData.timestamps,
              ...pumpHzHistoryData.timestamps
            ])].sort((a, b) => a - b);

            tempChart.data.labels = outsideTempTimestamps.map(t => new Date(t * 1000));
            tempChart.data.datasets[0].data = alignData(tempHistoryData.values, tempHistoryData.timestamps, outsideTempTimestamps);
            tempChart.data.datasets[1].data = alignData(outsideTempHistoryData.values, outsideTempHistoryData.timestamps, outsideTempTimestamps);
            tempChart.data.datasets[2].data = alignData(pumpHzHistoryData.values, pumpHzHistoryData.timestamps, outsideTempTimestamps);

            console.log("tempChart labels:", tempChart.data.labels);
            console.log("tempChart datasets:", tempChart.data.datasets);
            tempChart.update('none');
            tempChart.resize();
            console.log("tempChart updated with outdoor temp data");
            break;

          case 'pump_hz_history_update':
            console.log("Processing pump_hz_history_update");
            const pumpHzFiltered = filterDuplicates(data.pumpHzHistory, data.timestamps);
            pumpHzHistoryData = { values: pumpHzFiltered.values, timestamps: pumpHzFiltered.timestamps };
            console.log("pumpHzHistoryData updated:", pumpHzHistoryData);

            if (!tempChart) {
              console.error("tempChart not initialized!");
              break;
            }

            const pumpHzTimestamps = [...new Set([
              ...tempHistoryData.timestamps,
              ...outsideTempHistoryData.timestamps,
              ...pumpHzHistoryData.timestamps
            ])].sort((a, b) => a - b);

            tempChart.data.labels = pumpHzTimestamps.map(t => new Date(t * 1000));
            tempChart.data.datasets[0].data = alignData(tempHistoryData.values, tempHistoryData.timestamps, pumpHzTimestamps);
            tempChart.data.datasets[1].data = alignData(outsideTempHistoryData.values, outsideTempHistoryData.timestamps, pumpHzTimestamps);
            tempChart.data.datasets[2].data = alignData(pumpHzHistoryData.values, pumpHzHistoryData.timestamps, pumpHzTimestamps);

            console.log("tempChart labels:", tempChart.data.labels);
            console.log("tempChart datasets:", tempChart.data.datasets);
            tempChart.update('none');
            tempChart.resize();
            console.log("tempChart updated with pump Hz data");
            break;

          case 'voltage_history_update':
            console.log("Processing voltage_history_update");
            const voltageFiltered = filterDuplicates(data.voltageHistory, data.timestamps);
            voltageHistoryData = { values: voltageFiltered.values, timestamps: voltageFiltered.timestamps };
            console.log("voltageHistoryData updated:", voltageHistoryData);

            if (!voltageChart) {
              console.error("voltageChart not initialized!");
              break;
            }

            const voltageTimestamps = [...new Set([
              ...voltageHistoryData.timestamps,
              ...ampsHistoryData.timestamps
            ])].sort((a, b) => a - b);

            voltageChart.data.labels = voltageTimestamps.map(t => new Date(t * 1000));
            voltageChart.data.datasets[0].data = alignData(voltageHistoryData.values, voltageHistoryData.timestamps, voltageTimestamps);
            voltageChart.data.datasets[1].data = alignData(ampsHistoryData.values, ampsHistoryData.timestamps, voltageTimestamps);

            console.log("voltageChart labels:", voltageChart.data.labels);
            console.log("voltageChart datasets:", voltageChart.data.datasets);
            voltageChart.update('none');
            voltageChart.resize();
            console.log("voltageChart updated with voltage data");
            break;

          case 'amps_history_update':
            console.log("Processing amps_history_update");
            const ampsFiltered = filterDuplicates(data.ampsHistory.map(v => Math.round(v * 10) / 10), data.timestamps);
            ampsHistoryData = { values: ampsFiltered.values, timestamps: ampsFiltered.timestamps };
            console.log("ampsHistoryData updated:", ampsHistoryData);

            if (!voltageChart) {
              console.error("voltageChart not initialized!");
              break;
            }

            const ampsTimestamps = [...new Set([
              ...voltageHistoryData.timestamps,
              ...ampsHistoryData.timestamps
            ])].sort((a, b) => a - b);

            voltageChart.data.labels = ampsTimestamps.map(t => new Date(t * 1000));
            voltageChart.data.datasets[0].data = alignData(voltageHistoryData.values, voltageHistoryData.timestamps, ampsTimestamps);
            voltageChart.data.datasets[1].data = alignData(ampsHistoryData.values, ampsHistoryData.timestamps, ampsTimestamps);

            console.log("voltageChart labels:", voltageChart.data.labels);
            console.log("voltageChart datasets:", voltageChart.data.datasets);
            voltageChart.update('none');
            voltageChart.resize();
            console.log("voltageChart updated with amps data");
            break;

          case 'hourly_fuel_history_update':
            console.log("Processing hourly_fuel_history_update");
            hourlyFuelHistoryData = {
              completed: data.hourlyFuelHistory || [],
              timestamps: data.hourlyFuelTimestamps || [],
              accumulator: data.hourlyFuelAccumulator || 0
            };
            console.log("hourlyFuelHistoryData updated:", hourlyFuelHistoryData);

            if (!hourlyFuelChart) {
              console.error("hourlyFuelChart not initialized!");
              break;
            }

            const fuelLabels = hourlyFuelHistoryData.timestamps.map(ts => new Date(ts * 1000));
            hourlyFuelChart.data.labels = fuelLabels;
            hourlyFuelChart.data.datasets[0].data = hourlyFuelHistoryData.completed;

            if (hourlyFuelHistoryData.accumulator > 0) {
              const currentHourStart = new Date(Math.floor(Date.now() / 3600 / 1000) * 3600 * 1000);
              hourlyFuelChart.data.labels = [...fuelLabels, currentHourStart];
              hourlyFuelChart.data.datasets[1].data = [...new Array(hourlyFuelHistoryData.completed.length).fill(null), hourlyFuelHistoryData.accumulator];
            } else {
              hourlyFuelChart.data.datasets[1].data = [];
            }

            // Calculate and update total gallons (24hr Gals)
            const totalGallons = hourlyFuelHistoryData.completed.reduce((sum, value) => sum + value, 0) + hourlyFuelHistoryData.accumulator;
            document.getElementById("totalGal").textContent = totalGallons.toFixed(2) + " Gal";

            console.log("hourlyFuelChart labels:", hourlyFuelChart.data.labels);
            console.log("hourlyFuelChart datasets:", hourlyFuelChart.data.datasets);
            hourlyFuelChart.update('none');
            hourlyFuelChart.resize();
            console.log("hourlyFuelChart updated");
            break;

          case 'watt_hour_history_update':
            console.log("Processing watt_hour_history_update");
            wattHourHistoryData = {
              completed: data.wattHourHistory || [],
              timestamps: data.wattHourTimestamps || [],
              accumulator: data.wattHourAccumulator || 0
            };
            console.log("wattHourHistoryData updated:", wattHourHistoryData);

            if (!wattHourChart) {
              console.error("wattHourChart not initialized!");
              break;
            }

            const wattLabels = wattHourHistoryData.timestamps.map(ts => new Date(ts * 1000));
            wattHourChart.data.labels = wattLabels;
            wattHourChart.data.datasets[0].data = wattHourHistoryData.completed;

            if (wattHourHistoryData.accumulator > 0) {
              const currentHourStart = new Date(Math.floor(Date.now() / 3600 / 1000) * 3600 * 1000);
              wattHourChart.data.labels = [...wattLabels, currentHourStart];
              wattHourChart.data.datasets[1].data = [...new Array(wattHourHistoryData.completed.length).fill(null), wattHourHistoryData.accumulator];
            } else {
              wattHourChart.data.datasets[1].data = [];
            }

            // Calculate and update total watt-hours (24hr Wh)
            const totalWattHours = wattHourHistoryData.completed.reduce((sum, value) => sum + value, 0) + wattHourHistoryData.accumulator;
            document.getElementById("totalWh").textContent = totalWattHours.toFixed(2) + " W";

            console.log("wattHourChart labels:", wattHourChart.data.labels);
            console.log("wattHourChart datasets:", wattHourChart.data.datasets);
            wattHourChart.update('none');
            wattHourChart.resize();
            console.log("wattHourChart updated");
            break;

          default:
            console.log(`Unhandled event: ${eventName}`);
        }
      } catch (error) {
        console.error("Error processing SSE event:", error, "Raw data:", e.data);
      }
    }
  };

  evtSource.onopen = function() {
    console.log("EventSource connection opened");
    if (!hasGeolocationFailed) checkAndUpdateZipCode();
  };

  evtSource.onerror = function(e) {
    console.error("EventSource failed:", e);
  };

  console.log("Triggering initial geolocation check...");
  checkAndUpdateZipCode();
});