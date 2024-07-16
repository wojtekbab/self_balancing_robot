/**
 * @file web_server.cpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief
 * @version 0.1
 * @date 2024-07-15
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "web_server.hpp"
#include "globals.hpp"
#include "freertos/event_groups.h"
#include "defines.hpp"

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Control Panel</title>
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.3rem;}
    p {font-size: 1.9rem;}
    body {max-width: 400px; margin: 0px auto; padding-bottom: 25px; position: relative;}
    #joystickContainer {display: flex; justify-content: center; margin-top: 20px; height: 200px;}
    #joystick {width: 200px; height: 200px; position: relative;}
    .toggle {
      display: inline-block;
      margin: 10px;
      background-color: #f1f1f1;
      border: 1px solid #ccc;
      padding: 10px 20px;
      cursor: pointer;
      user-select: none;
    }
    .toggle.on {
      background-color: #4CAF50;
      color: white;
    }
    .toggle.off {
      background-color: #f44336;
      color: white;
    }
  </style>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.8.2/nipplejs.min.js"></script>
</head>
<body>
  <h2>Control Panel</h2>
  <p>
    <span id="x2Label">x2:</span> 
    <span id="textJoystickValueX">0</span>, 
    <span id="x6Label">x6:</span> 
    <span id="textJoystickValueY">0</span>
  </p>
  <div id="joystickContainer">
    <div id="joystick"></div>
  </div>

  <div id="toggleButton" class="toggle off" onclick="toggleButtonClicked()">OFF</div>

  <script>
    var joystick = nipplejs.create({
      zone: document.getElementById('joystick'),
      mode: 'static',
      position: {left: '50%', top: '50%'},
      color: 'blue',
      size: 200,
      catchDistance: 200, // Ensure the joystick area is covered
    });

    var last_x2 = 0;
    var last_x6 = 0;
    var threshold = 1; // Threshold for change in joystick value
    var sendDelay = 200; // Minimum delay between sending data (in ms)
    var lastSentTime = 0; // Timestamp of the last sent data

    function sign(x) {
      return x > 0 ? 1 : x < 0 ? -1 : 0;
    }

    function sendJoystickData(x, y) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/joystick?x=" + x + "&y=" + y, true);
      xhr.send();
      lastSentTime = Date.now(); // Update last sent time
    }

    joystick.on('move', function (evt, data) {
      var x = Math.round(data.instance.frontPosition.x);
      var y = Math.round(data.instance.frontPosition.y);

      // Reverse axis values and apply signum function
      var x2 = -y;
      var x6 = -x;

      // Only send data if the change exceeds the threshold
      if (Math.abs(x2 - last_x2) >= threshold || Math.abs(x6 - last_x6) >= threshold) {
        document.getElementById('textJoystickValueX').innerHTML = x2;
        document.getElementById('textJoystickValueY').innerHTML = x6;
        last_x2 = x2;
        last_x6 = x6;

        // Check if enough time has passed since the last data send
        var currentTime = Date.now();
        if (currentTime - lastSentTime >= sendDelay) {
          sendJoystickData(x2, x6);
        }
      }
    });

    joystick.on('end', function (evt, data) {
      document.getElementById('textJoystickValueX').innerHTML = "0";
      document.getElementById('textJoystickValueY').innerHTML = "0";
      sendJoystickData(0, 0);
      last_x2 = 0;
      last_x6 = 0;
    });

    function toggleButtonClicked() {
      var button = document.getElementById('toggleButton');
      var currentState = button.classList.contains('on');
      if (currentState) {
        button.classList.remove('on');
        button.classList.add('off');
        button.innerHTML = 'OFF';
        sendToggleData('off');
      } else {
        button.classList.remove('off');
        button.classList.add('on');
        button.innerHTML = 'ON';
        sendToggleData('on');
      }
    }

    function sendToggleData(state) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/toggle?state=" + state, true);
      xhr.send();
    }
  </script>
</body>
</html>
)rawliteral";

void setupWebServer()
{
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });

  // Read values send user web page
  server.on("/joystick", HTTP_GET, [](AsyncWebServerRequest *request)
            {
  Reference_velocities_struct reference_velocities_struct = {};
  reference_velocities_struct.reference_x_dot =   request->getParam("x")->value().toInt(); 
  reference_velocities_struct.reference_psi_dot =   request->getParam("y")->value().toInt(); 
  if(queue_reference_velocities != NULL)
  {
    xQueueOverwrite(queue_reference_velocities, &reference_velocities_struct);
  }
  request->send(200, "text/plain", "OK"); });

  // Route to handle toggle button state
  server.on("/toggle", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    String state = request->getParam("state")->value();
    EventBits_t uxBits = xEventGroupGetBits(xEventGroup);
    if (state == "on") 
    {
      xEventGroupSetBits(xEventGroup, START_STOP_BIT);
    } 
    else if (state == "off") 
    {
      xEventGroupClearBits(xEventGroup, START_STOP_BIT);
    }
    request->send(200, "text/plain", "OK"); });

  // Start server
  server.begin();
}
