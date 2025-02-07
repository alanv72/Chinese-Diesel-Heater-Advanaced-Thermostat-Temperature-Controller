#ifndef INDEX_HTML_H
#define INDEX_HTML_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <style>
    body {
      margin-top: 40px;
      background: #2c3e50 url('/fireside.jpg') no-repeat center center fixed; 
      background-size: cover;
      color: #f39c12; 
      font-family: 'Arial', sans-serif;
      text-shadow: 1px 1px 2px rgba(0, 0, 0, 0.5);
    }
    h1, h2 {
      color: #e67e22;
      text-align: center;
    }
    .slider {
      -webkit-appearance: none;
      width: 80%;
      height: 25px;
      background: linear-gradient(to right, yellow, orange, red, red); /* Static gradient from yellow to orange to red */
      outline: none;
      opacity: 0.7;
      -webkit-transition: .2s;
      transition: opacity .2s;
      position: relative;
    }

    /* Styling for the thumb */
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 25px;
      height: 25px;
      background: #e74c3c;
      cursor: pointer;
    }

    .slider::-moz-range-thumb {
      width: 25px;
      height: 25px;
      background: #e74c3c;
      cursor: pointer;
    }
    .floating-label {
      position: absolute;
      background: rgba(0, 0, 0, 0.7);
      color: white;
      padding: 4px 8px;
      border-radius: 4px;
      font-size: 12px;
      pointer-events: none;
      display: none;
      transform: translateX(-50%);
      white-space: nowrap;
      z-index: 1000;
    }
    button {
      background-color: #d35400;
      border: none;
      color: white;
      padding: 10px 24px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
      margin: 4px 2px;
      cursor: pointer;
      border-radius: 12px;
      transition: background-color 0.3s;
    }
    button:hover {
      background-color: #e67e22;
    }
    #status {
      background: rgba(0, 0, 0, 0.5);
      padding: 20px;
      border-radius: 10px;
      margin: 20px auto;
      max-width: 500px;
    }
    #currentSetTemp {
      position: relative;
      color: #f1c40f;
      font-size: 1.2em;
      text-align: center;
      margin-top: 20px;
    }
    .file-area {
      background: rgba(0, 0, 0, 0.5);
      padding: 20px;
      border-radius: 10px;
      margin: 20px auto;
      max-width: 500px;
    }
    table {
      width: 100%;
      border-collapse: collapse;
    }
    th, td {
      border: 1px solid #f39c12;
      padding: 8px;
      text-align: left;
    }
    th {
      background-color: #d35400;
    }
    #prime {
      display: none;
    }
    #shutdownHeater {
      position: fixed;
      top: 30px;
      right: 10px;
      z-index: 1001;
      background-color: #d35400;
      border: none;
      color: white;
      padding: 5px 10px; /* Halved padding */
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 12px; /* Smaller font size */
      cursor: pointer;
      border-radius: 4px; /* Halved border-radius */
      transition: background-color 0.3s;
      /* Touch-friendly adjustments */
      min-width: 50px; /* Half of previous min-width */
      min-height: 20px; /* Half of previous min-height */
    }

    #shutdownHeater:hover {
      background-color: #e67e22;
    }
  </style>
</head>
<body>
<button id="shutdownHeater" onclick="shutdownHeater()">Shut Down Heater</button>
<div id="statusBanner" style="position: fixed; top: 0; left: 0; width: 100%; background: rgba(0, 0, 0, 0.7); color: #f39c12; font-size: 12px; padding: 5px 0; text-align: center; z-index: 1000;">
  <span id="uptime" style="margin: 0 10px;">Uptime: <span></span></span>
  <span id="currentTime" style="margin: 0 10px;">Time: <span></span></span>
  <span id="currentDate" style="margin: 0 10px;">Date: <span></span></span>
  <span style="margin: 0 10px;">Temp: <span id="currentTemp">0</span></span>
  <span style="margin: 0 10px;">State: <span id="heaterState">Unknown</span></span>
  <span style="margin: 0 10px;">Hour Meter: <span id="runtime">0</span></span>
  <span style="margin: 0 10px;">Fan: <span id="fanSpeed">0</span></span>
  <span style="margin: 0 10px;">Volt: <span id="supplyVoltage">0.0</span></span>
  <span id="voltageWarning" style="margin: 0 10px;"></span>
  <span style="margin: 0 10px;">Internal Temp: <span id="heaterInternalTemp">0</span></span>
  <span style="margin: 0 10px;">Glow Plug Current: <span id="glowPlugCurrent">0</span> A</span>
  <span style="margin: 0 10px;">Pump Hz: <span id="pumpHz">0</span> Hz</span>
  <span style="margin: 0 10px;">Glow Plug: <span id="glowPlugHours">0.00</span></span>
</div>
  <h1>Heater Control</h1>
  <div id="message"></div>
  <div id="currentSetTemp">Set Temperature: <span id="setTemperatureDisplay">60</span>°F
    <input type="range" min="46" max="95" value="60" class="slider" id="tempSlider">
  <!--  <div id="sliderValue" class="floating-label"></div> -->
  </div>
  
  <div style="text-align: center;">
    <button onclick="setTemp(46)">Frost</button>
    <button onclick="setTemp(60)">Eco</button>
    <button onclick="setTemp(65)">Sleep</button>
    <button onclick="setTemp(71)">Comfort</button>
  </div>
  <div style="text-align: center;" id="prime">
    <button id="primeStart" onclick="primePump('start')">Start Pump Prime</button>
    <button id="primeStop" onclick="primePump('stop')">Stop Pump Prime</button>
  </div>
  <div style="text-align: center;">
  <input type="checkbox" id="frostMode" onchange="toggleFrostMode(this.checked)">
  <label for="frostMode">Frost Mode</label>
  </div>
  <div style="text-align: center;" class="tank">

    <h2>Fuel Consumption</h2>
    <p>Lifetime Fuel Used: <span id="lifetimeFuel">0</span></p>
    <p>Current Usage: <span id="currentUsage">0</span></p>
    <p>Average Gallons per Hour: <span id="avgGPH">0.00</span></p>
    <p>Fuel Used This Tank: <span id="tankFuel">0</span></p>
    <button onclick="resetAverage()">Reset Average</button><button onclick="resetTank()">Reset Tank</button>
    
    <h2>Tank Management</h2>
    <p>Current Tank Size: <span id="currentTankSize">0</span></p>
    <input type="number" id="tankSizeInput" min="0" step="0.1"><button onclick="setTankSize()">Set Tank Size</button>
    
  </div>
  <div class="file-area">
    <h2>File Upload</h2>
    <form method="POST" action="/upload" enctype="multipart/form-data">
      <input type="file" name="data"/>
      <input type="submit" name="upload" value="Upload" title="Upload File">
    </form>
    <p>Please wait after clicking upload; there's no progress indicator. The page will refresh to show the uploaded file.</p>
    <p>Files might not upload if too large or if the filename contains special characters.</p>
    <p>Check serial output for upload progress.</p>
  </div>

  <div id="fileListing">
    <h2>Files in SPIFFS:</h2>
    <button id="listFilesBtn">List Files</button>
    <table id="fileTable">
      <thead>
        <tr><th>Name</th><th>Size</th></tr>
      </thead>
      <tbody id="fileListBody"></tbody>
    </table>
  </div>

<script>
const ML_TO_GALLON = 0.000264172; // Conversion factor from ml to gallons

// Functions that need to be globally accessible for inline event handlers
function setTemp(value) {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/settemp");
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("temp=" + value);
  
  // Update the slider value
  var tempSlider = document.getElementById("tempSlider");
  if (tempSlider) {
    tempSlider.value = value;
  }
  
  // Update the display
  updateTempDisplay(value);
}

function updateTempDisplay(value) {
  var display = document.getElementById("setTemperatureDisplay");
  if (display) display.textContent = value;
  document.getElementById("currentSetTemp").querySelector("span").textContent = value;
}

function toggleFrostMode(enabled) {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/frostMode");
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.send("enable=" + enabled);
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

function resetAverage() {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/resetAverage");
  xhr.send();
}

function shutdownHeater() {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/shutdownHeater"); // Assuming this is your endpoint
  xhr.setRequestHeader("Content-Type", "application/x-www-form-urlencoded");
  xhr.onload = function() {
    if (xhr.status === 200) {
      console.log("Heater shutdown command sent");
      // Optionally, update UI to reflect this action or show a status message
      document.getElementById("message").textContent = "Heater shutdown command sent";
    } else {
      console.error("Failed to send heater shutdown command");
      document.getElementById("message").textContent = "Failed to shut down heater";
    }
  };
  xhr.send();
}

document.addEventListener('DOMContentLoaded', function() {
  var tempSlider = document.getElementById("tempSlider");
  var lastUpdateTime = 0; // Timestamp for the last heater_update update
  var intervalId; // For managing the interval

  if (tempSlider) {
    tempSlider.oninput = function() {
      var value = this.value; // Current slider value
      
      // Update both displays immediately when the slider moves
      updateTempDisplay(value);
    };

    tempSlider.onchange = function() {
      setTemp(this.value); // This will send the new temp to the server
    };
  }

  // Function to update slider and temp display based on heater_update data
  function updateSliderAndDisplay(data) {
    if (tempSlider) {
      tempSlider.value = data.setTemp.toFixed(0);
      updateTempDisplay(data.setTemp.toFixed(0));
    }
  }

  // Set slider on initial page load
  if (tempSlider) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/getcurrenttemp", true); // Assuming there's an endpoint to get current temp
    xhr.onload = function() {
      if (xhr.status === 200) {
        var currentTemp = JSON.parse(xhr.responseText).setTemp || 60; // Default to 60 if no data
        tempSlider.value = currentTemp;
        updateTempDisplay(currentTemp);
      }
    };
    xhr.send();
  }

  var evtSource = new EventSource("/events");
  
  evtSource.onmessage = function(e) {
    console.log("Event received:", e.data);
    if (e.data) {  // Check if e.data is not undefined
      var eventLines = e.data.split('\n');
      var eventName = eventLines[0].split(': ')[1];
      var eventData = eventLines[1].split(': ')[1];

      if (eventName === 'heater_update') {
        try {
          var data = JSON.parse(eventData);
          var currentTime = new Date().getTime(); // Current time in milliseconds

          // Check if a minute has passed since the last heater_update for setTemp updates
          if (currentTime - lastUpdateTime >= 60000) { // 60000 ms = 1 minute
            lastUpdateTime = currentTime; // Update the last update time

            // Update slider and display for setTemp
            updateSliderAndDisplay(data);
          }

          // Update everything else on every heater_update
          document.getElementById("currentTemp").textContent = data.currentTemp.toFixed(1) + "°F";
          document.getElementById("heaterState").textContent = data.state;
          document.getElementById("runtime").textContent = (data.heaterHourMeter).toFixed(2) + "Hrs";
          
          document.getElementById("currentTime").querySelector("span").textContent = data.time;
          document.getElementById("currentDate").querySelector("span").textContent = data.date;

          document.getElementById("lifetimeFuel").textContent = (data.fuelConsumedLifetime * ML_TO_GALLON).toFixed(2) + " Gal";
          document.getElementById("tankFuel").textContent = (data.fuelConsumedTank * ML_TO_GALLON).toFixed(2) + " Gal";
          document.getElementById("currentUsage").textContent = (data.currentUsage * ML_TO_GALLON).toFixed(2) + " GPH";
          document.getElementById("currentTankSize").textContent = data.tankSizeGallons.toFixed(0) + "Gal";
          document.getElementById("avgGPH").textContent = data.averageGPH.toFixed(2) + "GPH";
          document.getElementById("fanSpeed").textContent = data.fanSpeed + "RPM";
          document.getElementById("supplyVoltage").textContent = data.supplyVoltage.toFixed(1) + "V";
          document.getElementById("voltageWarning").textContent = data.voltageWarning;
          document.getElementById("glowPlugHours").textContent = data.glowPlugHours.toFixed(2) + "Hrs";
          document.getElementById("frostMode").checked = data.frostMode;

          // Update new fields
          document.getElementById("heaterInternalTemp").textContent = data.heaterinternalTemp.toFixed(1) + "°F";
          document.getElementById("glowPlugCurrent").textContent = data.glowPlugCurrent_Amps.toFixed(2);
          document.getElementById("pumpHz").textContent = data.pumpHz.toFixed(2);

          var fuelUsed = data.fuelConsumedTank * ML_TO_GALLON; // Convert to gallons if necessary
          var tankSize = data.tankSizeGallons;
          
          var messageDiv = document.getElementById("message");
          
          if (tankSize > 0 && fuelUsed >= tankSize * 0.75) {
            messageDiv.textContent = "Warning: Fuel level is low!";
          } else {
            messageDiv.textContent = "";
          }

          // Start or reset the interval for periodic updates of setTemp
          clearInterval(intervalId);
          intervalId = setInterval(() => {
            // Fetch current temp from server every minute if not updated by heater_update for setTemp
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/getcurrenttemp", true);
            xhr.onload = function() {
              if (xhr.status === 200) {
                var tempData = JSON.parse(xhr.responseText);
                updateSliderAndDisplay(tempData);
              }
            };
            xhr.send();
          }, 60000);

        } catch(error) {
          console.error("Failed to parse JSON:", error);
        }
      } else {
        console.log("Received unknown event:", eventName);
      }
    } else {
      console.error("Received an event with no data.");
    }
  };

  var listFilesBtn = document.getElementById('listFilesBtn');
  if (listFilesBtn) {
    listFilesBtn.addEventListener('click', function() {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/listfiles", true);
      xhr.onload = function() {
        if (xhr.status === 200) {
          var fileList = JSON.parse(xhr.responseText);
          updateFileList(fileList.files);
        }
      };
      xhr.send();
    });
  }

  function updateFileList(data) {
    var tableBody = document.getElementById('fileListBody');
    if (tableBody) {
      tableBody.innerHTML = ''; // Clear existing entries
      data.forEach(function(file) {
        var row = tableBody.insertRow();
        var nameCell = row.insertCell(0);
        var sizeCell = row.insertCell(1);
        nameCell.textContent = file.name;
        sizeCell.textContent = file.size;
      });
    }
  }

  evtSource.onerror = function(e) {
    console.error("EventSource failed:", e);
  };

  evtSource.onopen = function(e) {
    console.log("EventSource connection opened");
  };
});
</script>
</body>
</html>
)rawliteral";

#endif // INDEX_HTML_H