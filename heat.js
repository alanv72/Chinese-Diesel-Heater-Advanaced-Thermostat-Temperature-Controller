const ML_TO_GALLON = 0.000264172;
let fanControlStates = { duct: false, wall: false };
let isSliderActive = false; // Tracks if slider is being adjusted
let tempAdjusting = false; // Tracks ESP32's tempadjusting state
let pendingSetTemp = null; // Stores the user-selected setTemp during adjustment
let isShuttingDown = false;

const QUICK_SET_TEMPS = {
  "Frost": 46,
  "Eco": 61,
  "Sleep": 66,
  "Comfort": 72
};

function drawGauge(canvasId, percent) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) {
    console.error(`Canvas ${canvasId} not found`);
    return;
  }
  const ctx = canvas.getContext('2d');
  const width = canvas.width; // 132
  const height = canvas.height; // 110
  const centerX = width / 2; // 66
  const centerY = height * 0.75; // 82.5 (110 * 0.75)
  const lineWidth = 19.8;
  const radius = Math.min(width, height) * 0.4 - lineWidth / 2; // ~39.6px radius
  const startAngle = Math.PI; // Bottom (180°)
  const maxAngle = Math.PI; // 180° arc

  ctx.clearRect(0, 0, width, height);

  // Background arc (gray)
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, startAngle, startAngle + maxAngle);
  ctx.lineWidth = lineWidth;
  ctx.strokeStyle = '#555';
  ctx.stroke();

  // Fill arc (orange to red gradient)
  const fillAngle = (percent / 100) * Math.PI;
  ctx.beginPath();
  ctx.arc(centerX, centerY, radius, startAngle, startAngle + fillAngle);
  const gradient = ctx.createLinearGradient(centerX - radius, centerY, centerX + radius, centerY);
  gradient.addColorStop(0, '#FFA500'); // Orange
  gradient.addColorStop(1, '#FF4500'); // Red
  ctx.lineWidth = lineWidth;
  ctx.strokeStyle = gradient;
  ctx.stroke();
}

function updateFanSpeed(fanType, percent) {
  percent = Math.round(parseFloat(percent) / 5) * 5; // Snap to 5% increments
  if (percent < 0 || percent > 100) return;

  drawGauge(fanType + 'FanGauge', percent);
  document.getElementById(fanType + 'FanGaugeValue').textContent = percent + '%';

  if (fanControlStates[fanType]) {
    fetch('/setFanSpeed', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: `${fanType}Speed=${percent}` // Send percent as expected by backend
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

  if (distance > radius * 1.5) {
    return; // Ignore clicks significantly outside the arc
  }

  let angle = Math.atan2(y, x);
  if (angle < 0) angle += 2 * Math.PI;
  angle = (angle > Math.PI) ? (angle - Math.PI) : (Math.PI - angle);
  angle = Math.max(0, Math.min(Math.PI, angle));
  const percent = Math.round((angle / Math.PI) * 100); // 0-100%

  updateFanSpeed(fanType, percent);
}

function drawGauge(canvasId, percent) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const width = canvas.width; // 132
  const height = canvas.height; // 110
  const centerX = width / 2; // 66
  const centerY = height * 0.75; // 82.5
  const lineWidth = 19.8;
  const radius = Math.min(width, height) * 0.4 - lineWidth / 2; // ~39.6px
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

function toggleFanControl(fanType) {
  fanControlStates[fanType] = !fanControlStates[fanType];
  const button = document.getElementById(fanType + 'FanControlToggle');
  
  // Update button state
  if (fanControlStates[fanType]) {
    button.textContent = 'Manual';
    button.classList.remove('active');
    button.style.backgroundColor = '#d35400';
  } else {
    button.textContent = 'Auto';
    button.classList.add('active');
    button.style.backgroundColor = '#e67e22';
  }

  // Ensure button remains clickable
  button.disabled = false;
  button.style.pointerEvents = 'auto';

  // Send mode change to server
  fetch('/setFanControlMode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: `fan=${fanType}&mode=${fanControlStates[fanType] ? 'manual' : 'auto'}`
  })
  .then(response => response.text())
  .then(data => console.log(data))
  .catch(error => {
    console.error('Error:', error);
    // Revert state on failure to keep UI consistent
    fanControlStates[fanType] = !fanControlStates[fanType];
    updateButtonState(fanType);
  });
}

// Helper to sync button state
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

  pendingSetTemp = value; // Set pending temp for adjustment
  updateTempDisplay(pendingSetTemp); // Immediately reflect selected value
  var tempSlider = document.getElementById("tempSlider");
  if (tempSlider) tempSlider.value = value;

  // Highlight the clicked button immediately
  if (button) {
    highlightQuickSetButton(value, button);
    lastClickedButton = button; // Store the clicked button
  }

  // Clear shutdown message if it exists
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
  if (tempSlider && !isSliderActive) { // Only check isSliderActive, not tempAdjusting
    tempSlider.value = data.setTemp.toFixed(0);
    if (!tempAdjusting || pendingSetTemp === null) {
      // Update display only if not adjusting or no pending value
      updateTempDisplay(data.setTemp.toFixed(0));
    }
  }
}

function highlightQuickSetButton(setTemp, clickedButton = null) {
  const buttons = document.querySelectorAll("#currentSetTemp + div button");
  const roundedSetTemp = Math.round(setTemp); // Round setTemp for comparison
  buttons.forEach(button => {
    const buttonTemp = QUICK_SET_TEMPS[button.textContent];
    if (clickedButton && button === clickedButton && tempAdjusting) {
      button.style.backgroundColor = "#e67e22"; // Keep clicked button active during adjustment
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

      // Clear shutdown message if it exists
      var messageDiv = document.getElementById("message");
      if (messageDiv.style.display === "inline-block" && messageDiv.textContent === "Heater was shut down.") {
        messageDiv.textContent = "";
        messageDiv.style.display = "none";
        isShuttingDown = false;
        console.log("Shutdown message cleared by frost mode toggle");
      }
    } else {
      console.error("Failed to toggle frost mode. Status: " + xhr.status);
      document.getElementById("frostModeEnable").checked = !newState; // Revert checkbox on failure
    }
  };

  xhr.onerror = function() {
    console.error("Network error while toggling frost mode");
    document.getElementById("frostModeEnable").checked = !newState; // Revert checkbox on network error
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

      // Clear shutdown message if it exists
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
      isShuttingDown = false; // Reset shutdown flag
      controlEnable = 1; // Assuming you want thermostat on
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
    // Find the end of the current duplicate sequence
    while (i + 1 < values.length && values[i + 1] === values[startIdx]) {
      i++;
    }
    // Add the first point of the sequence
    filtered.push({ value: values[startIdx], timestamp: timestamps[startIdx] });
    // Add the last point of the sequence if it’s more than one point
    if (i > startIdx) {
      filtered.push({ value: values[i], timestamp: timestamps[i] });
    }
    i++; // Move past the last point of this sequence
  }
  // Ensure the last point is included if not already
  if (filtered.length === 0 || filtered[filtered.length - 1].timestamp !== timestamps[timestamps.length - 1]) {
    filtered.push({ value: values[values.length - 1], timestamp: timestamps[timestamps.length - 1] });
  }
  return {
    values: filtered.map(item => item.value),
    timestamps: filtered.map(item => item.timestamp)
  };
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
        listfiles(); // Refresh list using lowercase function
      })
      .catch(error => console.error('Error deleting file:', error));
  }
}

document.addEventListener('DOMContentLoaded', function() {
  var tempSlider = document.getElementById("tempSlider");
  var currentSetTemp = document.getElementById("currentSetTemp");
  let lastClickedButton = null; // Track the last clicked button
  let serverZipCode = "64856"; // Default until we get the first event
  let hasGeolocationFailed = false;

  // Function to fetch zip code from coordinates
  function fetchZipCodeFromCoords(lat, lon) {
    console.log(`Fetching zip code for lat: ${lat}, lon: ${lon}`);
    const apiKey = "aeb9ccaba969c927fc2b8ce501da53a8";
    const url = `http://api.openweathermap.org/geo/1.0/reverse?lat=${lat}&lon=${lon}&limit=1&appid=${apiKey}`;

    return fetch(url)
      .then(response => {
        if (!response.ok) {
          throw new Error(`HTTP error! Status: ${response.status}`);
        }
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

  // Function to send zip code to server
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

  // Function to manually set zip code
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

  // Function to check and update zip code
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
          case error.PERMISSION_DENIED:
            errorMsg += "User denied the request for Geolocation.";
            break;
          case error.POSITION_UNAVAILABLE:
            errorMsg += "Location information is unavailable.";
            break;
          case error.TIMEOUT:
            errorMsg += "The request to get user location timed out.";
            break;
          default:
            errorMsg += "An unknown error occurred.";
        }
        console.error(errorMsg);
        showManualInput(errorMsg);
      },
      { timeout: 10000 } // 10-second timeout
    );
  }

  // Function to show manual input on failure
  function showManualInput(errorMessage) {
    if (!hasGeolocationFailed) {
      hasGeolocationFailed = true;
      console.log("Showing manual input due to:", errorMessage);
      document.getElementById("zipErrorMessage").textContent = errorMessage;
      document.getElementById("manualZipInput").style.display = "block";
    }
  }

  document.getElementById("setZipButton").addEventListener("click", manualSetZipCode);

  // Initialize gauges
  drawGauge('ductFanGauge', 0);
  drawGauge('wallFanGauge', 0);

  // Add click event listeners to gauges
  document.getElementById('ductFanGauge').addEventListener('click', (e) => handleGaugeInteraction('duct', e));
  document.getElementById('wallFanGauge').addEventListener('click', (e) => handleGaugeInteraction('wall', e));

  // Set initial button states (no listeners needed since HTML has onclick)
  const ductToggleButton = document.getElementById('ductFanControlToggle');
  const wallToggleButton = document.getElementById('wallFanControlToggle');

  if (ductToggleButton) {
    updateButtonState('duct'); // Set initial UI state
  } else {
    console.error('ductFanControlToggle button not found in DOM');
  }

  if (wallToggleButton) {
    updateButtonState('wall'); // Set initial UI state
  } else {
    console.error('wallFanControlToggle button not found in DOM');
  }

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

  // Initialize tempChart
var ctx = document.getElementById('tempChart').getContext('2d');
tempChart = new Chart(ctx, {
  type: 'line',
  data: {
    labels: [],
    datasets: [
      {
        label: 'Indoor Temp (°F)',
        data: [],
        fill: false,
        borderColor: 'orange',
        tension: 0.1,
        yAxisID: 'y',
        spanGaps: true
      },
      {
        label: 'Outdoor Temp (°F)',
        data: [],
        fill: false,
        borderColor: 'blue',
        tension: 0.1,
        yAxisID: 'y',
        spanGaps: true
      },
      {
        label: 'Pump Hz',
        data: [],
        fill: false,
        borderColor: 'green',
        tension: 0.1,
        yAxisID: 'y1',
        spanGaps: true
      }
    ]
  },
  options: {
    scales: {
      x: {
        type: 'time',
        time: {
          unit: 'minute',
          displayFormats: { minute: 'H:mm' },
          tooltipFormat: 'HH:mm'
        },
        adapters: { date: { locale: customLocale } },
        title: { display: false, text: 'Time' }
      },
      y: {
        title: { display: true, text: 'Temperature (°F)', color: 'white' },
        position: 'left',
        ticks: {
          stepSize: 2,
          callback: function(value) { return value + '°F'; },
          color: 'grey'
        },
        afterDataLimits: function(scale) {
          const indoorTemps = scale.chart.data.datasets[0].data;
          const outdoorTemps = scale.chart.data.datasets[1].data;
          // Filter out null values for scaling
          const allTemps = [...indoorTemps, ...outdoorTemps].filter(v => v !== null && !isNaN(v));
          if (allTemps.length === 0) return;
          const dataMin = Math.min(...allTemps);
          const dataMax = Math.max(...allTemps);
          const midpoint = (dataMin + dataMax) / 2;
          const range = dataMax - dataMin;
          let min, max;
          if (range < 12) {
            min = midpoint - 6 - 1; // Add 1°F buffer below
            max = midpoint + 6 + 1; // Add 1°F buffer above
          } else {
            min = dataMin - 1; // Add 1°F buffer below
            max = dataMax + 1; // Add 1°F buffer above
          }
          min = Math.floor(min / 2) * 2; // Round down to nearest 2°F
          max = Math.ceil(max / 2) * 2;   // Round up to nearest 2°F
          // Explicitly set min and max to enforce the scaling
          scale.options.min = min;
          scale.options.max = max;
        },
        grid: { color: 'rgba(255, 255, 255, 0.1)' }
      },
      y1: {
        title: { display: true, text: 'Pump Frequency (Hz)', color: 'white' },
        position: 'right',
        min: -1,
        max: 6,
        ticks: {
          stepSize: 1,
          callback: function(value) { return value + ' Hz'; },
          color: 'grey'
        },
        grid: { drawOnChartArea: false }
      }
    },
    plugins: { legend: { labels: { color: 'grey' } } }
  },
  plugins: [{
    beforeDraw: (chart) => {
      const ctx = chart.canvas.getContext('2d');
      ctx.save();
      ctx.globalCompositeOperation = 'destination-over';
      ctx.fillStyle = 'lightyellow';
      ctx.fillRect(0, 0, chart.width, chart.height);
      ctx.restore();
    }
  }]
});

var ctxVoltage = document.getElementById('voltageChart').getContext('2d');
var voltageChart = new Chart(ctxVoltage, {
  type: 'line',
  data: {
    labels: [],
    datasets: [
      {
        label: 'Voltage (V)',
        data: [],
        fill: false,
        borderColor: 'red',
        tension: 0.1,
        spanGaps: true,
        yAxisID: 'y-voltage'
      },
      {
        label: 'Current (A)',
        data: [],
        fill: false,
        borderColor: 'blue',
        tension: 0.1,
        spanGaps: true,
        yAxisID: 'y-amps'
      }
    ]
  },
  options: {
    scales: {
      x: {
        type: 'time',
        time: { unit: 'minute', displayFormats: { minute: 'H:mm' }, tooltipFormat: 'HH:mm' },
        adapters: { date: { locale: customLocale } },
        title: { display: false, text: 'Time', color: 'grey' }
      },
      'y-voltage': {
        title: { display: false, text: 'Voltage (V)', color: 'grey' },
        position: 'left',
        ticks: { stepSize: 0.5, callback: function(value) { return value + 'V'; }, color: 'grey' },
        afterDataLimits: function(scale) {
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
        title: { display: false, text: 'Current (A)', color: 'grey' }, // Changed display to true for consistency
        position: 'right',
        ticks: { 
          stepSize: 1, // Adjusted step size for finer granularity
          callback: function(value) { return value + 'A'; },
          color: 'grey'
        },
        afterDataLimits: function(scale) {
          const amps = scale.chart.data.datasets[1].data.filter(v => v !== null && !isNaN(v));
          if (amps.length === 0) {
            scale.options.min = 0;
            scale.options.max = 5; // Default range if no data
            return;
          }
          const dataMin = Math.min(...amps);
          const dataMax = Math.max(...amps);
          const range = dataMax - dataMin;
          let min = 0; // Always start at 0 as requested
          let max = range < 3 ? dataMax + 1.5 : dataMax + 0.5; // Buffer above max
          max = Math.ceil(max / 1) * 1; // Round up to nearest integer
          if (max < 5) max = 5; // Minimum reasonable max
          scale.options.min = min;
          scale.options.max = max;
        },
        grid: { drawOnChartArea: false }
      }
    },
    plugins: { legend: { labels: { color: 'grey' } } }
  },
  plugins: [{
    beforeDraw: (chart) => {
      const ctx = chart.canvas.getContext('2d');
      ctx.save();
      ctx.globalCompositeOperation = 'destination-over';
      ctx.fillStyle = 'lightyellow';
      ctx.fillRect(0, 0, chart.width, chart.height);
      ctx.restore();
    }
  }]
});

  // New Hourly Fuel Chart
  var ctxHourlyFuel = document.getElementById('hourlyFuelChart').getContext('2d');
  var hourlyFuelChart = new Chart(ctxHourlyFuel, {
    type: 'bar',
    data: {
      labels: [],
      datasets: [
        {
          label: 'Gal Per HR',
          data: [],
          backgroundColor: 'rgba(255, 165, 0, 0.7)',
          borderColor: 'rgba(255, 165, 0, 1)',
          borderWidth: 1
        },
        {
          label: 'Current Hour (In Progress)',
          data: [],
          backgroundColor: 'rgba(255, 165, 0, 0.3)',
          borderColor: 'rgba(255, 165, 0, 1)',
          borderWidth: 1
        }
      ]
    },
    options: {
      scales: {
        x: {
          type: 'time',
          time: {
            unit: 'hour',
            displayFormats: { hour: 'H' },
            tooltipFormat: 'MM/dd H:00'
          },
          adapters: { date: { locale: customLocale } },
          title: { display: true, text: 'Hour', color: 'grey' }
        },
        y: {
          beginAtZero: true,
          title: { display: false, text: 'Gallons', color: 'grey' },
          ticks: {
            stepSize: 0.005,
            callback: function(value) { return value.toFixed(2) + ' gal'; },
            color: 'grey'
          },
          afterDataLimits: function(scale) {
            const completedData = scale.chart.data.datasets[0].data;
            const currentData = scale.chart.data.datasets[1].data;
            const allData = [...completedData, ...currentData].filter(v => v !== null && !isNaN(v));
            if (allData.length === 0) return;
            const max = Math.max(...allData);
            scale.options.max = Math.ceil(max * 1.1 / 0.05) * 0.05;
          }
        }
      },
      plugins: { legend: { labels: { color: 'grey' } } }
    },
    plugins: [{
      beforeDraw: (chart) => {
        const ctx = chart.canvas.getContext('2d');
        ctx.save();
        ctx.globalCompositeOperation = 'destination-over';
        ctx.fillStyle = 'lightyellow';
        ctx.fillRect(0, 0, chart.width, chart.height);
        ctx.restore();
      }
    }]
  });

var ctxWattHour = document.getElementById('wattHourChart').getContext('2d');
var wattHourChart = new Chart(ctxWattHour, {
  type: 'bar',
  data: {
    labels: [],
    datasets: [
      {
        label: 'Watt-HR',
        data: [],
        backgroundColor: 'rgba(0, 128, 255, 0.7)',
        borderColor: 'rgba(0, 128, 255, 1)',
        borderWidth: 1
      },
      {
        label: 'Watt-HR (In Progress)',
        data: [],
        backgroundColor: 'rgba(0, 128, 255, 0.3)',
        borderColor: 'rgba(0, 128, 255, 1)',
        borderWidth: 1
      }
    ]
  },
  options: {
    scales: {
      x: {
        type: 'time',
        time: {
          unit: 'hour',
          displayFormats: { hour: 'H' },
          tooltipFormat: 'MM/dd H:00'
        },
        adapters: { date: { locale: customLocale } },
        title: { display: true, text: 'Hour', color: 'grey' }
      },
      y: {
        beginAtZero: true,
        title: { display: false, text: 'Watt-Hours', color: 'grey' },
        ticks: {
          stepSize: 10,
          callback: function(value) { return value.toFixed(0) + ' Wh'; },
          color: 'grey'
        },
        afterDataLimits: function(scale) {
          const completedData = scale.chart.data.datasets[0].data;
          const currentData = scale.chart.data.datasets[1].data;
          const allData = [...completedData, ...currentData].filter(v => v !== null && !isNaN(v));
          if (allData.length === 0) return;
          const max = Math.max(...allData);
          scale.options.max = Math.ceil(max * 1.1 / 10) * 10;
        }
      }
    },
    plugins: { legend: { labels: { color: 'grey' } } }
  },
  plugins: [{
    beforeDraw: (chart) => {
      const ctx = chart.canvas.getContext('2d');
      ctx.save();
      ctx.globalCompositeOperation = 'destination-over';
      ctx.fillStyle = 'lightyellow';
      ctx.fillRect(0, 0, chart.width, chart.height);
      ctx.restore();
    }
  }]
});

  // Slider interaction handling
  if (tempSlider) {
    // Start adjusting (ignore events)
    tempSlider.addEventListener('mousedown', function() {
      isSliderActive = true;
      console.log("Slider adjustment started");
    });
    tempSlider.addEventListener('touchstart', function() {
      isSliderActive = true;
      console.log("Slider adjustment started (touch)");
    });

    // Update display live while sliding
    tempSlider.oninput = function() {
      var value = this.value;
      updateTempDisplay(value);
    };

    // Finish adjusting (send new setTemp and lock display)
    tempSlider.addEventListener('mouseup', function() {
      isSliderActive = false;
      pendingSetTemp = this.value; // Store the user-selected value
      setTemp(pendingSetTemp, null); // No button to highlight
      updateTempDisplay(pendingSetTemp); // Lock display to this value
      console.log("Slider adjustment ended");
    });
    tempSlider.addEventListener('touchend', function() {
      isSliderActive = false;
      pendingSetTemp = this.value; // Store the user-selected value
      setTemp(pendingSetTemp, null); // No button to highlight
      updateTempDisplay(pendingSetTemp); // Lock display to this value
      console.log("Slider adjustment ended (touch)");
    });
  }

  // File listing and deletion
  var listFilesBtn = document.getElementById('listFilesBtn');
  if (listFilesBtn) {
    listFilesBtn.addEventListener('click', listfiles); // Changed to listfiles
  }

  function listfiles() { // Changed from listFiles to listfiles
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/listfiles", true); // Lowercase endpoint
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
        var nameCell = row.insertCell(0);
        var sizeCell = row.insertCell(1);
        var actionCell = row.insertCell(2);
        nameCell.textContent = file.name;
        sizeCell.textContent = file.size;
        actionCell.innerHTML = `<div id="delete"><button class="delete-btn" onclick="deleteFile('${file.name}')">Delete</button></div>`;
      });
    }
  }

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

        if (eventName === 'heater_update') {
          var data = JSON.parse(eventData);
          var currentTime = new Date().getTime();

        if (data.zipcode) {
          serverZipCode = data.zipcode;
          document.getElementById("currentZipCode").textContent = serverZipCode;
          // console.log("Updated serverZipCode from event:", serverZipCode);
        }

          updateSliderAndDisplay(data);

          // Handle tempadjusting state
          tempAdjusting = data.tempadjusting || false;
          if (tempAdjusting) {
            currentSetTemp.classList.add('adjusting');
            updateTempDisplay(data.targettemp !== undefined ? data.targettemp : (pendingSetTemp !== null ? pendingSetTemp : data.setTemp.toFixed(0)));
            // Highlight based on targettemp during adjustment, if available
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
            updateTempDisplay(data.setTemp.toFixed(0));
            highlightQuickSetButton(data.setTemp); // Highlight based on setTemp post-adjustment
          }

          // Create Date object from UTC epoch time (assuming epochtime is in seconds)
          const utcDateTime = new Date(data.epochTime * 1000);

          // Convert to client's local timezone with formatting options
          const timeOptions = {
              hour: '2-digit',
              minute: '2-digit',
              second: '2-digit',
              hour12: true
          };

          const dateOptions = {
              year: 'numeric',
              month: '2-digit',
              day: '2-digit'
          };

          // Get local time and date strings
          const localTime = utcDateTime.toLocaleTimeString([], timeOptions);
          const localDate = utcDateTime.toLocaleDateString([], dateOptions);

          // Update the DOM elements
          document.getElementById("currentTime").querySelector("span").textContent = localTime;
          document.getElementById("currentDate").querySelector("span").textContent = localDate;

          document.getElementById("currentTemp").textContent = data.currentTemp + "°F";
          document.getElementById("heaterState").textContent = data.state;
          document.getElementById("runtime").textContent = data.heaterHourMeter.toFixed(2) + "Hrs";
          //document.getElementById("currentTime").querySelector("span").textContent = data.time;
          //document.getElementById("currentDate").querySelector("span").textContent = data.date;
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
          document.getElementById("tankruntime").textContent = (data.remainingRuntimeHours == null? '∞' : data.remainingRuntimeHours.toFixed(2)) + "Hrs";

          // Display tempwarn based on integer value
          const tempWarnElement = document.getElementById("tempWarn");
          const heaterErrorElement = document.getElementById("heatererror");
          const heaterErrorTxtElement = document.getElementById("heatererrortxt");

          // Handle temperature warnings
          if (tempWarnElement) {
              switch (data.tempwarn) {
                  case 3:
                      tempWarnElement.textContent = "HEATER INTERNAL TEMP OUT OF SPEC!";
                      tempWarnElement.style.display = "inline-block";
                      tempWarnElement.style.color = "red";
                      break;
                  case 2:
                      tempWarnElement.textContent = "Shut down. Wall Temp > 120F!";
                      tempWarnElement.style.display = "inline-block";
                      tempWarnElement.style.color = "red";
                      break;
                  case 1:
                      tempWarnElement.textContent = "Wall temp > 110F.";
                      tempWarnElement.style.display = "inline-block";
                      tempWarnElement.style.color = "orange";
                      break;
                  case 0:
                  default:
                      tempWarnElement.textContent = "";
                      tempWarnElement.style.display = "none";
                      break;
              }
          }

          // Handle heater error display based on either condition
          if (heaterErrorElement && heaterErrorTxtElement) {
              // Set error text if errornum > 1
              heaterErrorTxtElement.textContent = (data.errornum > 1) 
                  ? (data.error || "Unknown Error") 
                  : "";

              // Show heatererror if errornum > 1 OR tempwarn > 0
              heaterErrorElement.style.display = (data.errornum > 1 || data.tempwarn > 0) 
                  ? "inline-block" 
                  : "none";
          }

          const supplyVoltage = (data.supplyVoltage <= 5.0 || data.supplyVoltage > 15.0 || isNaN(data.supplyVoltage)) ? 12.0 : data.supplyVoltage;
          const voltageRange = supplyVoltage - 10.0;

          fanControlStates.duct = data.ductFanManualControl;
          updateButtonState('duct');
          if (fanControlStates.duct) {
              const ductPwm = data.manualDuctFanSpeed;
              let ductVoltage = data.manualDuctFanVoltage;
              let ductPercent;
              if (ductPwm === 0) {
                  ductPercent = 0; // Off state
                  ductVoltage = 0.0; // Ensure voltage reflects off state
              } else {
                  // Calculate percent based on voltage (10.0V to supplyVoltage)
                  const ductVoltageRange = supplyVoltage - 10.0;
                  ductPercent = ((ductVoltage - 10.0) / ductVoltageRange) * 95 + 5; // Raw percent (5-100%)
                  ductPercent = Math.round(ductPercent / 5) * 5; // Snap to nearest 5%
                  ductPercent = Math.max(0, Math.min(100, ductPercent)); // Clamp 0-100%
                  // Correct voltage if mismatch with PWM
                  if (ductVoltage === 0 && ductPwm > 0) {
                      ductVoltage = (ductPwm / 1023) * supplyVoltage;
                      console.log(`Corrected duct voltage from 0 to ${ductVoltage.toFixed(1)}V for PWM ${ductPwm}`);
                  }
              }
              drawGauge('ductFanGauge', ductPercent);
              document.getElementById('ductFanGaugeValue').textContent = ductPercent + '%';
              document.getElementById('ductfan').textContent = ductPwm === 0 ? "Man Off" : `${ductVoltage.toFixed(1)}V (${ductPwm})`;
          } else {
              // Auto mode: Gauge at 0%, but text updates with voltage/PWM
              const ductPwm = data.ductfan; // PWM value
              let ductVoltage;
              if (ductPwm === 0) {
                  ductVoltage = 0.0;
              } else {
                  // Map PWM back to voltage (10.0V to supplyVoltage)
                  const ductVoltageRange = supplyVoltage - 10.0;
                  ductVoltage = 10.0 + (ductPwm / 1023) * ductVoltageRange;
                  ductVoltage = Math.min(supplyVoltage, Math.max(10.0, ductVoltage)); // Clamp to valid range
              }
              drawGauge('ductFanGauge', 0); // Gauge at 0% in auto mode
              document.getElementById('ductFanGaugeValue').textContent = '0%';
              document.getElementById('ductfan').textContent = ductPwm === 0 ? "Off" : `${ductVoltage.toFixed(1)}V (${ductPwm})`;
          }

          fanControlStates.wall = data.wallFanManualControl;
          updateButtonState('wall');
          if (fanControlStates.wall) {
              const wallPwm = data.manualWallFanSpeed;
              let wallVoltage = data.manualWallFanVoltage;
              let wallPercent;
              if (wallPwm === 0) {
                  wallPercent = 0; // Off state
                  wallVoltage = 0.0; // Ensure voltage reflects off state
              } else {
                  // Calculate percent based on voltage (6.7V to supplyVoltage)
                  const wallVoltageRange = supplyVoltage - 6.7;
                  wallPercent = ((wallVoltage - 6.7) / wallVoltageRange) * 95 + 5; // Raw percent (5-100%)
                  wallPercent = Math.round(wallPercent / 5) * 5; // Snap to nearest 5%
                  wallPercent = Math.max(0, Math.min(100, wallPercent)); // Clamp 0-100%
                  // Correct voltage if mismatch with PWM
                  if (wallVoltage === 0 && wallPwm > 0) {
                      wallVoltage = (wallPwm / 1023) * supplyVoltage;
                      console.log(`Corrected wall voltage from 0 to ${wallVoltage.toFixed(1)}V for PWM ${wallPwm}`);
                  }
              }
              drawGauge('wallFanGauge', wallPercent);
              document.getElementById('wallFanGaugeValue').textContent = wallPercent + '%';
              document.getElementById('wallfan').textContent = wallPwm === 0 ? "Man Off" : `${wallVoltage.toFixed(1)}V (${wallPwm})`;
          } else {
              // Auto mode: Gauge at 0%, but text updates with voltage/PWM
              const wallPwm = data.wallfan; // PWM value
              let wallVoltage;
              if (wallPwm === 0) {
                  wallVoltage = 0.0;
              } else {
                  // Map PWM back to voltage (6.7V to supplyVoltage)
                  const wallVoltageRange = supplyVoltage - 6.7;
                  wallVoltage = 6.7 + (wallPwm / 1023) * wallVoltageRange;
                  wallVoltage = Math.min(supplyVoltage, Math.max(6.7, wallVoltage)); // Clamp to valid range
              }
              drawGauge('wallFanGauge', 0); // Gauge at 0% in auto mode
              document.getElementById('wallFanGaugeValue').textContent = '0%';
              document.getElementById('wallfan').textContent = wallPwm === 0 ? "Off" : `${wallVoltage.toFixed(1)}V (${wallPwm})`;
          }

          // 1. Fuel Gallons Total (24hr Gals)
          if (data.hourlyFuelHistory) {
            const hourlyFuelData = JSON.parse(data.hourlyFuelHistory);
            const hourlyFuelHistory = hourlyFuelData.hourlyFuelHistory || [];
            const hourlyFuelAccumulator = hourlyFuelData.hourlyFuelAccumulator || 0;
            
            // Sum historical gallons and add accumulator
            const totalGallons = hourlyFuelHistory.reduce((sum, value) => sum + value, 0) + hourlyFuelAccumulator;
            document.getElementById("totalGal").textContent = totalGallons.toFixed(2) + " Gal";
          }

          // 2. Watt-Hours Total (24hr Wh)
          if (data.wattHourHistory) {
            const wattHourData = JSON.parse(data.wattHourHistory);
            const hourlyWattHours = wattHourData.wattHours || [];
            const wattHourAccumulator = wattHourData.wattHourAccumulator || 0;
            
            // Sum historical watt-hours and add accumulator
            const totalWattHours = hourlyWattHours.reduce((sum, value) => sum + value, 0) + wattHourAccumulator;
            document.getElementById("totalWh").textContent = totalWattHours.toFixed(2) + " Wh";
          }


          var thermostatLabel = document.getElementById("thermostatLabel");
          thermostatLabel.textContent = data.controlEnable ? "Thermostat On" : "Thermostat Off";
          thermostatLabel.classList.toggle("active", data.controlEnable);
          document.getElementById("thermostatEnable").checked = data.controlEnable;
          // Frost Mode update (new)
          var frostModeLabel = document.getElementById("frostModeLabel");
          frostModeLabel.textContent = data.frostMode ? "Frost Mode On" : "Frost Mode Off";
          frostModeLabel.classList.toggle("active", data.frostMode);
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
          var rotation = (fuelPercentage / 100) * 180;
          document.getElementById("fuelGaugeFill").style.transform = `rotate(${rotation}deg)`;
          var fuelGaugeFill = document.getElementById("fuelGaugeFill");
          if (fuelPercentage < 10) fuelGaugeFill.style.backgroundColor = "#FF0000";
          else if (fuelPercentage < 25) fuelGaugeFill.style.backgroundColor = "#FFA500";
          else if (fuelPercentage < 50) fuelGaugeFill.style.backgroundColor = "#FFFF";
          else fuelGaugeFill.style.backgroundColor = "#327A24";

          // Outdoor weather
          let outdoorValue = "";
          if (data.outsideTempF !== undefined && data.outsideHumidity !== undefined) {
            outdoorValue = `${data.outsideTempF.toFixed(0)}°F / ${data.outsideHumidity.toFixed(0)}%`;
          } else if (data.outsideTempF !== undefined) {
            outdoorValue = `${data.outsideTempF.toFixed(1)}°F / N/A%`;
          } else if (data.outsideHumidity !== undefined) {
            outdoorValue = `N/A°F / ${data.outsideHumidity.toFixed(1)}%`;
          } else {
            outdoorValue = "N/A";
          }
          document.getElementById('outdoor').innerText = outdoorValue;

          // Handle shutdown state
          var messageDiv = document.getElementById("message");
          if (isShuttingDown && data.statenum === 0) {
            messageDiv.textContent = "Heater was shut down.";
            messageDiv.style.display = "inline-block";
          } else if (!isShuttingDown && messageDiv.textContent === "Heater was shut down.") {
            // Don't clear here; let controls clear it
          } else if (!isShuttingDown && tankSizeGallons > 0 && fuelConsumedTank >= tankSizeGallons * 0.90) {
            messageDiv.textContent = "Warning: Fuel level is low!";
            messageDiv.style.display = "inline-block";
          } else if (!isShuttingDown) {
            messageDiv.textContent = "";
            messageDiv.style.display = "none";
          }

          // Update hourly fuel chart
          if (data.hourlyFuelHistory) {
            const hourlyFuelData = JSON.parse(data.hourlyFuelHistory);
            const hourlyFuel = hourlyFuelData.hourlyFuelHistory || [];
            const hourlyTimestamps = hourlyFuelData.hourlyFuelTimestamps || [];
            const accumulator = hourlyFuelData.hourlyFuelAccumulator || 0;

            // Use absolute epoch timestamps (seconds to milliseconds)
            const labels = hourlyTimestamps.map(ts => new Date(ts * 1000));
            
            hourlyFuelChart.data.labels = labels;
            hourlyFuelChart.data.datasets[0].data = hourlyFuel;

            // Add current hour
            const currentHourStart = new Date(Math.floor(data.epochTime / 3600) * 3600 * 1000);
            hourlyFuelChart.data.labels = [...labels, currentHourStart];
            hourlyFuelChart.data.datasets[1].data = [...new Array(hourlyFuel.length).fill(null), accumulator];

            hourlyFuelChart.update();
          }

          // Update watt-hour chart
          if (data.wattHourHistory) {
            const wattHourData = JSON.parse(data.wattHourHistory);
            const hourlyWattHours = wattHourData.wattHours || [];
            const wattHourTimestamps = wattHourData.wattHourTimestamps || [];
            const wattHourAccumulator = wattHourData.wattHourAccumulator || 0;

            // Use absolute epoch timestamps (seconds to milliseconds)
            const labels = wattHourTimestamps.map(ts => new Date(ts * 1000));
            
            wattHourChart.data.labels = labels;
            wattHourChart.data.datasets[0].data = hourlyWattHours;

            const currentHourStart = new Date(Math.floor(data.epochTime / 3600) * 3600 * 1000);
            if (wattHourAccumulator > 0) {
              wattHourChart.data.labels = [...labels, currentHourStart];
              wattHourChart.data.datasets[1].data = [...new Array(hourlyWattHours.length).fill(null), wattHourAccumulator];
            } else {
              wattHourChart.data.datasets[1].data = [];
            }

            wattHourChart.update();
          }
          document.getElementById("avgWattHours24h").textContent = data.avgWattHours24h.toFixed(2) + " Wh/Hr";

          // Update tempChart with sparsified datasets
          if (tempChart) {
            const tempHistory = data.tempHistory ? JSON.parse(data.tempHistory) : { tempHistory: [], timestamps: [] };
            const outsideTempHistory = data.outsideTempHistory ? JSON.parse(data.outsideTempHistory) : { outsideTempHistory: [], timestamps: [] };
            const pumpHzHistory = data.pumpHzHistory ? JSON.parse(data.pumpHzHistory) : { pumpHzHistory: [], timestamps: [] };

            // Determine the master timeline
            const timelines = [
              { name: 'tempHistory', timestamps: tempHistory.timestamps },
              { name: 'outsideTempHistory', timestamps: outsideTempHistory.timestamps },
              { name: 'pumpHzHistory', timestamps: pumpHzHistory.timestamps }
            ];
            const masterTimeline = timelines.reduce((max, current) => 
              current.timestamps.length > max.timestamps.length ? current : max, 
              timelines[0]
            );

            // Use absolute epoch timestamps (seconds to milliseconds)
            tempChart.data.labels = masterTimeline.timestamps.map(t => new Date(t * 1000));

            const filteredIndoor = filterDuplicates(tempHistory.tempHistory, tempHistory.timestamps);
            const filteredOutdoor = filterDuplicates(outsideTempHistory.outsideTempHistory, outsideTempHistory.timestamps);
            const filteredPumpHz = filterDuplicates(pumpHzHistory.pumpHzHistory, pumpHzHistory.timestamps);

            const alignData = (filteredValues, filteredTimestamps, masterTimestamps) => {
              const data = new Array(masterTimestamps.length).fill(null);
              filteredTimestamps.forEach((timestamp, index) => {
                const idx = masterTimestamps.indexOf(timestamp);
                if (idx !== -1) data[idx] = filteredValues[index];
              });
              return data;
            };

            tempChart.data.datasets[0].data = alignData(filteredIndoor.values, filteredIndoor.timestamps, masterTimeline.timestamps);
            tempChart.data.datasets[1].data = alignData(filteredOutdoor.values, filteredOutdoor.timestamps, masterTimeline.timestamps);
            tempChart.data.datasets[2].data = alignData(filteredPumpHz.values, filteredPumpHz.timestamps, masterTimeline.timestamps);

            tempChart.update();
          }

          // Update voltage chart (including amps)
          if (data.voltageHistory || data.ampsHistory) {
            const voltageHistory = data.voltageHistory ? JSON.parse(data.voltageHistory) : { voltageHistory: [], timestamps: [] };
            const ampsHistory = data.ampsHistory ? JSON.parse(data.ampsHistory) : { ampsHistory: [], timestamps: [] };
            ampsHistory.ampsHistory = ampsHistory.ampsHistory.map(value => Math.round(value * 10) / 10);

            // Combine timestamps and use absolute epoch times
            const allTimestamps = [...new Set([...voltageHistory.timestamps, ...ampsHistory.timestamps])].sort((a, b) => a - b);
            voltageChart.data.labels = allTimestamps.map(t => new Date(t * 1000));

            const filteredVoltage = filterDuplicates(voltageHistory.voltageHistory, voltageHistory.timestamps);
            const filteredAmps = filterDuplicates(ampsHistory.ampsHistory, ampsHistory.timestamps);

            const alignData = (filteredValues, filteredTimestamps, masterTimestamps) => {
              const data = new Array(masterTimestamps.length).fill(null);
              filteredTimestamps.forEach((timestamp, index) => {
                const idx = masterTimestamps.indexOf(timestamp);
                if (idx !== -1) data[idx] = filteredValues[index];
              });
              return data;
            };

            voltageChart.data.datasets[0].data = alignData(filteredVoltage.values, filteredVoltage.timestamps, allTimestamps);
            voltageChart.data.datasets[1].data = alignData(filteredAmps.values, filteredAmps.timestamps, allTimestamps);

            voltageChart.update();
          }          
        }
      } catch (error) {
        console.error("Error processing SSE event:", error, "Raw data:", e.data);
      }
    }
  };

  evtSource.onopen = function() {
    console.log("EventSource connection opened");
    // Trigger geolocation check on first connect
    if (!hasGeolocationFailed) {
      checkAndUpdateZipCode();
    }
  };

  evtSource.onerror = function(e) {
    console.error("EventSource failed:", e);
  };

  evtSource.onopen = function(e) {
    console.log("EventSource connection opened");
  };

  // Trigger geolocation check immediately for debugging
  console.log("Triggering initial geolocation check...");
  checkAndUpdateZipCode();
});