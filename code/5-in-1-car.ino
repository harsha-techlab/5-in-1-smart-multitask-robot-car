/*
   5-in-1 Smart Robot Car 
   Target: Customized ESP32-S3 (Station Mode)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// --- Function Prototypes (Forward Declarations) ---
int calculateDynamicThreshold(int speed);
void moveForward(int speed);
void moveBackward(int speed);
void spinLeft(int speed);
void spinRight(int speed);
void stopMotors();
long measureDistance();
int scanDirection(int angle);
void executeDriftManeuver();
void runObstacleAvoidance();
void runLineFollowing();
void runHandFollowing();
void yieldDelay(unsigned long ms);

// --- Your Custom Pin Assignments ---
const int pinENA = 45;   // Left motor speed
const int pinIN1 = 48;   // Left motor dir 1
const int pinIN2 = 47;   // Left motor dir 2
const int pinIN3 = 21;   // Right motor dir 1
const int pinIN4 = 14;   // Right motor dir 2
const int pinENB = 13;   // Right motor speed

const int pinServo = 12; // Servo control pin
const int pinTrig = 11;  // Ultrasonic Trigger
const int pinEcho = 10;  // Ultrasonic Echo (via voltage divider)

const int pinIR1 = 15;   // Left IR Sensor
const int pinIR2 = 16;   // Right IR Sensor

// --- Central Mode Configuration ---
enum RobotMode {
  MODE_MANUAL,
  MODE_OBSTACLE,
  MODE_LINE,
  MODE_GESTURE
};
RobotMode currentMode = MODE_MANUAL;

// --- Speed & Calibration Parameters ---
const int AUTO_DRIVE_SPEED = 95;   // Safe, calibrated speed for autonomous navigation
const int TURN_SPEED = 220;        // High-torque turning speed
const int TURN_DURATION = 850;     // Calibrated turn duration (ms)
const int PIVOT_TIME = 800;        // Hand-tracking pivot duration (ms)

// Mutable manual speed driven by the slider (Default 95, ranges 70-140)
int activeDriveSpeed = 95;             

const int BASE_SPEED = 70;         // Adjusted Slow speed limit
const int MAX_SPEED = 120;         // Adjusted Fast speed limit
const int BASE_THRESHOLD = 30;     
const int MAX_THRESHOLD = 65;      
int activeObstacleThreshold;       // Calculated dynamically in setup

// --- Hand Tracking Parameters ---
const int DIST_TOO_CLOSE = 12;     
const int DIST_START_FOLLOW = 22;  
const int DIST_MAX_FOLLOW = 50;    

// --- Servo Angles ---
const int ANGLE_CENTER = 135;      // Calibrated center
const int ANGLE_LEFT = 180;        
const int ANGLE_RIGHT = 90;        
const int SERVO_DELAY = 400;       

// Filtering variables
int consecutiveErrors = 0;
long lastGoodDistance = 100;
int absenceCounter = 0;
const int ABSENCE_THRESHOLD = 5;

// --- Stunt Configurations ---
const int DRIFT_CHARGE_SPEED = 130;    // Speed for the initial 20cm run
const int DRIFT_SPIN_SPEED = 255;      // Maximum torque for the 360-degree pivot
const int DRIFT_CHARGE_DURATION = 750; // Time (ms) to travel 20cm
const int DRIFT_SPIN_DURATION = 1000;  // Time (ms) to execute the complete spin

// Web Server Port 80
WebServer server(80);

Servo headServo;

// --- HTML/CSS Web Dashboard with Joystick & Landscape Layout ---
const char* htmlDashboard = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Smart Robot Car Dashboard</title>
    <style>
        /* Base page layout */
        body, html { 
            font-family: Arial, sans-serif; 
            text-align: center; 
            background-color: #1a1a1f; 
            color: white; 
            margin: 0; 
            padding: 0; 
            overflow: hidden; 
            height: 100vh; 
        }
        
        /* Block text selection and callouts globally */
        body, html, .btn-mode, .btn, #joy-base, #joy-knob, .slider, .drawer-item {
            -webkit-touch-callout: none !important; 
            -webkit-user-select: none !important;   
            -khtml-user-select: none !important;     
            -moz-user-select: none !important;      
            -ms-user-select: none !important;       
            user-select: none !important;           
        }

        h1 { margin: 10px 0; font-size: 20px; color: #ffa500; text-transform: uppercase; letter-spacing: 2px; }
        
        /* Master Layout Container */
        .dashboard-container { display: flex; flex-direction: column; align-items: center; justify-content: center; height: calc(100vh - 40px); box-sizing: border-box; padding: 5px 15px; }
        
        /* Joystick Section */
        .joystick-panel { display: flex; justify-content: center; align-items: center; margin: 10px 0; }
        #joy-base { position: relative; width: 170px; height: 170px; background: radial-gradient(circle, #2a2a35 40%, #15151b 100%); border: 3px solid #ffa500; border-radius: 50%; touch-action: none; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
        #joy-knob { position: absolute; top: 50px; left: 50px; width: 70px; height: 70px; background: radial-gradient(circle, #ffa500 20%, #cc8400 100%); border-radius: 50%; box-shadow: 0 2px 10px rgba(0,0,0,0.6); }

        /* Speed Control Slider */
        .slider-panel { width: 100%; max-width: 260px; margin: 5px 0 15px 0; }
        .slider-label { font-size: 14px; color: #aaa; margin-bottom: 5px; font-weight: bold; }
        .slider { -webkit-appearance: none; width: 100%; height: 8px; border-radius: 5px; background: #33333d; outline: none; margin: 0; }
        .slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #ffa500; cursor: pointer; border: 2px solid #fff; }

        /* Mode Selector Panel */
        .mode-container { display: flex; flex-direction: column; gap: 8px; width: 100%; max-width: 260px; }
        .btn-mode { padding: 10px; background-color: #2b2b36; border: 2px solid #ffa500; border-radius: 8px; color: #ffa500; font-size: 13px; font-weight: bold; cursor: pointer; transition: 0.1s; width: 100%; box-sizing: border-box; }
        .btn-mode:active, .active-mode { background-color: #ffa500; color: #1a1a1f; }
        
        /* Action buttons */
        .btn-drift { background: linear-gradient(135deg, #b22222 0%, #8b0000 100%); border: 2px solid #ff3333; color: white; font-size: 14px; letter-spacing: 1px; }
        .btn-drift:active { background: #ff0000; transform: scale(0.95); }

        /* Glowing Voice Microphone Button */
        .btn-voice { background: linear-gradient(135deg, #1e90ff 0%, #00008b 100%); border: 2px solid #00bfff; color: white; }
        .btn-voice.listening { background: linear-gradient(135deg, #ff4500 0%, #b22222 100%); border-color: #ff0000; box-shadow: 0 0 15px #ff0000; }

        /* Floating HUD Telemetry */
        .telemetry-panel { position: absolute; top: 15px; right: 15px; font-size: 11px; background: rgba(26,26,36,0.8); border: 1px solid #333; padding: 6px 15px; border-radius: 8px; color: #888; display: flex; flex-direction: column; align-items: flex-end; gap: 4px; }

        /* Warning Banner */
        .chrome-warn { display: none; font-size: 10px; color: #ff3333; background: rgba(255,0,0,0.1); border: 1px dashed #ff3333; padding: 5px; border-radius: 5px; margin: 5px auto; max-width: 280px; }

        /* --- LANDSCAPE LAYOUT RESPONSIVENESS --- */
        @media (orientation: landscape) {
            h1 { font-size: 18px; margin: 5px 0; }
            .dashboard-container { flex-direction: row; justify-content: space-around; width: 100%; height: calc(100vh - 30px); padding: 5px 40px; }
            .joystick-panel { margin: 0; }
            #joy-base { width: 150px; height: 150px; }
            #joy-knob { width: 60px; height: 60px; top: 45px; left: 45px; }
            .right-column { display: flex; flex-direction: column; width: 50%; max-width: 320px; gap: 10px; }
            .slider-panel { max-width: 100%; margin: 0; }
            .mode-container { width: 100%; max-width: 100%; gap: 6px; }
            .btn-mode { padding: 10px; font-size: 13px; }
            .chrome-warn { max-width: 100%; margin: 0; }
        }
    </style>
</head>
<body>
    <h1>Smart Robot Car</h1>
    
    <!-- Sidebar Toggle Menu -->
    <div id="menu-btn" onclick="toggleMenu(true)" style="position: absolute; top: 15px; left: 15px; z-index: 800; width: 44px; height: 44px; background: rgba(26,26,36,0.8); border: 1px solid #ffa500; border-radius: 8px; display: flex; align-items: center; justify-content: center; font-size: 20px; color: #ffa500; cursor: pointer;">&#9776;</div>

    <!-- Top-Left Menu Drawer -->
    <div id="menu-drawer" style="display: none; flex-direction: column; position: fixed; top: 0; left: 0; width: 220px; height: 100vh; background: rgba(11,11,14,0.95); border-right: 1px solid #ffa500; z-index: 850; padding-top: 70px; text-align: left; box-sizing: border-box;">
        <div style="position: absolute; top: 15px; left: 15px; font-size: 22px; color: #b22222; cursor: pointer;" onclick="toggleMenu(false)">&times;</div>
        <a href="#" class="drawer-item" onclick="triggerMode('/mode_manual', 'Manual Steering')">MANUAL DRIVE</a>
        <a href="#" class="drawer-item" onclick="triggerMode('/mode_obstacle', 'Obstacle Avoidance')">AUTO OBSTACLE</a>
        <a href="#" class="drawer-item" onclick="triggerMode('/mode_line', 'Line Following')">LINE FOLLOW</a>
        <a href="#" class="drawer-item" onclick="triggerMode('/mode_gesture', 'Hand Following')">HAND TRACK</a>
    </div>

    <!-- Floating HUD Telemetry -->
    <div class="telemetry-panel">
        <div>MODE: <b id="mode-telemetry" style="color: #ffa500;">MANUAL</b></div>
        <div id="vocal-hud" style="color: #00bfff; font-weight: bold; font-size: 9px;">VOCAL: STANDBY</div>
    </div>

    <div class="dashboard-container">
        <!-- Left Side: Joystick -->
        <div class="joystick-panel">
            <div id="joy-base">
                <div id="joy-knob"></div>
            </div>
        </div>

        <!-- Right Side: Controls & Sliders -->
        <div class="right-column">
            <!-- Warning block for secure context issues -->
            <div id="chrome-warn-banner" class="chrome-warn">⚠️ Mic access blocked. App permissions denied OR check chrome://flags is enabled.</div>

            <!-- Speed Slider -->
            <div class="slider-panel">
                <div class="slider-label">SPEED: <span id="speed-val">95</span></div>
                <input type="range" min="70" max="140" value="95" class="slider" id="speed-slider" oninput="updateSpeed(this.value)">
            </div>

            <!-- Mode Selector Panel -->
            <div class="mode-container">
                <button id="btn_voice" class="btn-mode btn-voice" onclick="toggleVoiceRecognition()">🎙️ TAP TO TALK</button>
                <button id="btn_drift" class="btn-mode btn-drift" onclick="triggerDrift()">DRIFT MANEUVER</button>
                <button id="m_manual" class="btn-mode active-mode" onclick="changeMode('/mode_manual')">Manual Steering</button>
                <button id="m_obstacle" class="btn-mode" onclick="changeMode('/mode_obstacle')">Obstacle Avoidance</button>
                <button id="m_line" class="btn-mode" onclick="changeMode('/mode_line')">Line Following</button>
                <button id="m_gesture" class="btn-mode" onclick="changeMode('/mode_gesture')">Hand Following</button>
            </div>
        </div>
    </div>

    <!-- Settings Overlay -->
    <div id="settings-overlay">
        <div class="settings-card">
            <h3>Manual Controller Settings</h3>
            <div class="radio-row">
                <input type="radio" id="o-landscape" name="orientation" checked onclick="setOrientation(true)">
                <label for="o-landscape">Force Landscape Layout</label>
            </div>
            <div class="radio-row">
                <input type="radio" id="o-portrait" name="orientation" onclick="setOrientation(false)">
                <label for="o-portrait">Portrait Adaptive Layout</label>
            </div>
            <button class="btn-settings-close" onclick="closeSettings()">SAVE & APPLY</button>
        </div>
    </div>

    <script>
        const base = document.getElementById('joy-base');
        const knob = document.getElementById('joy-knob');
        const speedVal = document.getElementById('speed-val');
        const voiceBtn = document.getElementById('btn_voice');
        const vocalHud = document.getElementById('vocal-hud');
        let baseRect = base.getBoundingClientRect();
        
        let activeState = "STOP"; 

        window.addEventListener('resize', () => {
            baseRect = base.getBoundingClientRect();
        });

        // HTML5 Web Speech API Setup (Tap to Start / Tap to Stop)
        let SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
        let recognition = null;
        let isListening = false;

        if (SpeechRecognition) {
            recognition = new SpeechRecognition();
            recognition.continuous = false;
            recognition.interimResults = false;
            recognition.lang = 'en-US';

            recognition.onstart = () => {
                isListening = true;
                voiceBtn.classList.add('listening');
                voiceBtn.innerText = "🎙️ LISTENING...";
                vocalHud.innerText = "VOCAL: LISTENING...";
            };

            recognition.onend = () => {
                isListening = false;
                voiceBtn.classList.remove('listening');
                voiceBtn.innerText = "🎙️ TAP TO TALK";
            };

            recognition.onerror = (e) => {
                console.error("Speech Error:", e);
                vocalHud.innerText = "VOCAL: BLOCKED/ERROR";
                if (e.error === 'not-allowed') {
                    document.getElementById('chrome-warn-banner').style.display = 'block';
                }
            };

            recognition.onresult = (event) => {
                const command = event.results[0][0].transcript.toLowerCase();
                vocalHud.innerText = "HEARD: \"" + command.toUpperCase() + "\"";
                processVoiceCommand(command);
            };
        } else {
            voiceBtn.style.display = 'none';
        }

        function toggleVoiceRecognition() {
            if (!recognition) return;
            if (isListening) {
                recognition.stop();
            } else {
                recognition.start();
            }
        }

        function processVoiceCommand(phrase) {
            // Phonetic Tolerance Parser (Accepts alternative spellings like "forword" or "backword")
            if (phrase.includes("forward") || phrase.includes("forword") || phrase.includes("go straight") || phrase.includes("go")) {
                sendDirection("FORWARD");
            } 
            else if (phrase.includes("backward") || phrase.includes("backword") || phrase.includes("reverse") || phrase.includes("go back") || phrase.includes("back")) {
                sendDirection("BACKWARD");
            } 
            else if (phrase.includes("left") || phrase.includes("turn left")) {
                sendDirection("LEFT");
            } 
            else if (phrase.includes("right") || phrase.includes("write") || phrase.includes("rite") || phrase.includes("turn right")) {
                sendDirection("RIGHT");
            } 
            else if (phrase.includes("stop") || phrase.includes("brake") || phrase.includes("halt") || phrase.includes("top")) {
                sendDirection("STOP");
            } 
            else if (phrase.includes("drift") || phrase.includes("slide")) {
                triggerDrift();
            } 
            else if (phrase.includes("boost") || phrase.includes("speed")) {
                fetch('/boost');
            } 
            else if (phrase.includes("obstacle") || phrase.includes("avoid")) {
                changeMode('/mode_obstacle');
            } 
            else if (phrase.includes("line") || phrase.includes("follow")) {
                changeMode('/mode_line');
            } 
            else if (phrase.includes("gesture") || phrase.includes("hand")) {
                changeMode('/mode_gesture');
            }
            else if (phrase.includes("manual")) {
                changeMode('/mode_manual');
            }
        }

        base.addEventListener('touchstart', handleStart, {passive: false});
        base.addEventListener('touchmove', handleMove, {passive: false});
        base.addEventListener('touchend', handleEnd, {passive: false});

        function handleStart(e) {
            e.preventDefault();
            baseRect = base.getBoundingClientRect(); 
        }

        function handleMove(e) {
            e.preventDefault();
            const touch = e.touches[0];
            const centerX = baseRect.left + baseRect.width / 2;
            const centerY = baseRect.top + baseRect.height / 2;
            
            let x = touch.clientX - centerX;
            let y = touch.clientY - centerY;
            
            const maxDistance = baseRect.width / 2 - knob.clientWidth / 2;
            const distance = Math.sqrt(x*x + y*y);

            if (distance > maxDistance) {
                const angle = Math.atan2(y, x);
                x = Math.cos(angle) * maxDistance;
                y = Math.sin(angle) * maxDistance;
            }

            knob.style.transform = `translate(${x}px, ${y}px)`;
            evaluateDirection(x, y, maxDistance);
        }

        // Reset state and stop car on release
        function handleEnd(e) {
            e.preventDefault();
            knob.style.transform = `translate(0px, 0px)`;
            sendDirection("STOP");
        }

        function evaluateDirection(x, y, maxDist) {
            const threshold = maxDist * 0.35; 
            if (Math.abs(x) < threshold && Math.abs(y) < threshold) {
                sendDirection("STOP");
                return;
            }

            if (Math.abs(y) > Math.abs(x)) {
                if (y < 0) {
                    sendDirection("FORWARD");
                } else {
                    sendDirection("BACKWARD");
                }
            } else {
                if (x < 0) {
                    sendDirection("LEFT");
                } else {
                    sendDirection("RIGHT");
                }
            }
        }

        function sendDirection(state) {
            if (activeState !== state) {
                activeState = state;
                let path = "/stop";
                if (state === "FORWARD") path = "/forward";
                if (state === "BACKWARD") path = "/backward";
                if (state === "LEFT") path = "/left";
                if (state === "RIGHT") path = "/right";
                
                if (state !== "STOP") {
                    highlightMode('m_manual');
                }
                fetch(path);
            }
        }

        function updateSpeed(val) {
            speedVal.innerText = val;
            fetch(`/set_speed?val=${val}`);
        }

        // Action: Trigger drift sequence
        function triggerDrift() {
            highlightMode('m_manual');
            fetch('/drift');
        }

        function toggleMenu(show) {
            document.getElementById('menu-drawer').style.display = show ? 'flex' : 'none';
        }

        // Configuration drawer controls
        function openSettings() {
            toggleMenu(false);
            document.getElementById('settings-overlay').style.display = 'flex';
        }

        function closeSettings() {
            document.getElementById('settings-overlay').style.display = 'none';
        }

        function setOrientation(isLandscape) {
            let container = document.querySelector('.dashboard-container');
            if (isLandscape) {
                container.style.flexDirection = 'row';
            } else {
                container.style.flexDirection = 'column';
            }
        }

        function changeMode(path) {
            fetch(path);
            let modeId = 'm_manual';
            let label = "Manual";
            if (path === '/mode_obstacle') { modeId = 'm_obstacle'; label = "Obstacle"; }
            if (path === '/mode_line') { modeId = 'm_line'; label = "Line Follow"; }
            if (path === '/mode_gesture') { modeId = 'm_gesture'; label = "Hand Follow"; }
            highlightMode(modeId);
            document.getElementById('mode-telemetry').innerText = label;
        }

        function triggerMode(path, modeName) {
            fetch(path);
            toggleMenu(false);
            document.getElementById('mode-telemetry').innerText = modeName;
            
            let modeId = 'm_manual';
            if (path === '/mode_obstacle') modeId = 'm_obstacle';
            if (path === '/mode_line') modeId = 'm_line';
            if (path === '/mode_gesture') modeId = 'm_gesture';
            highlightMode(modeId);
        }

        function highlightMode(activeId) {
            document.querySelectorAll('.btn-mode').forEach(btn => {
                btn.classList.remove('active-mode');
            });
            document.getElementById(activeId).classList.add('active-mode');
        }
    </script>
</body>
</html>
)rawliteral";

// --- Dynamic Threshold Calculation ---
int calculateDynamicThreshold(int speed) {
  if (speed <= BASE_SPEED) return BASE_THRESHOLD;
  if (speed >= MAX_SPEED) return MAX_THRESHOLD;
  return BASE_THRESHOLD + ((speed - BASE_SPEED) * (MAX_THRESHOLD - BASE_THRESHOLD)) / (MAX_SPEED - BASE_SPEED);
}

// --- Non-Blocking Yield Delay Helper ---
void yieldDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    server.handleClient();
    delay(1); 
  }
}

// --- Basic Motor Drives ---
void moveForward(int speed) {
  analogWrite(pinENA, speed);
  analogWrite(pinENB, speed);
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4, LOW);
}

void moveBackward(int speed) {
  analogWrite(pinENA, speed);
  analogWrite(pinENB, speed);
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, HIGH);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, HIGH);
}

void spinLeft(int speed) {
  analogWrite(pinENA, speed);
  analogWrite(pinENB, speed);
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, HIGH);
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4, LOW);
}

void spinRight(int speed) {
  analogWrite(pinENA, speed);
  analogWrite(pinENB, speed);
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, HIGH);
}

void stopMotors() {
  analogWrite(pinENA, 0);
  analogWrite(pinENB, 0);
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, LOW);
}

// --- Dynamic Distance Helper (Single Probe) ---
long measureDistance() {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);

  long duration = pulseIn(pinEcho, HIGH, 15000); 
  if (duration == 0) return -1;
  return (duration * 0.0343) / 2;
}

// --- Dynamic Rotation Probe ---
int scanDirection(int angle) {
  if (headServo.attached() == false) headServo.attach(pinServo, 500, 2400);
  headServo.write(angle);
  yieldDelay(350); 
  long distance = measureDistance();
  if (distance == -1) {
    yieldDelay(50); 
    distance = measureDistance();
  }
  return distance;
}

// --- Automated Drift Maneuver ---
void executeDriftManeuver() {
  Serial.println("Executing 360 Drift...");
  moveForward(DRIFT_CHARGE_SPEED);
  yieldDelay(DRIFT_CHARGE_DURATION); 
  spinLeft(DRIFT_SPIN_SPEED);
  yieldDelay(DRIFT_SPIN_DURATION);   
  stopMotors();
}

// --- Autopilot: Obstacle Avoidance ---
void runObstacleAvoidance() {
  if (currentMode != MODE_OBSTACLE) return;
  if (headServo.attached() == false) headServo.attach(pinServo, 500, 2400);

  long currentDistance = measureDistance();
  if (currentMode != MODE_OBSTACLE) return;

  if (currentDistance == -1) {
    consecutiveErrors++;
    if (consecutiveErrors >= 3) {
      stopMotors();
      return;
    }
    currentDistance = lastGoodDistance;
  } else {
    consecutiveErrors = 0;
    lastGoodDistance = currentDistance;
  }

  if (currentDistance > activeObstacleThreshold) {
    moveForward(AUTO_DRIVE_SPEED);
    yieldDelay(30); 
    if (currentMode != MODE_OBSTACLE) { stopMotors(); return; }
  } 
  else {
    stopMotors();
    yieldDelay(400); 
    if (currentMode != MODE_OBSTACLE) { stopMotors(); return; }

    if (currentDistance > 0 && currentDistance < 18) {
      moveBackward(BASE_SPEED);
      yieldDelay(750); 
      if (currentMode != MODE_OBSTACLE) { stopMotors(); return; }
      stopMotors();
      yieldDelay(250); 
      if (currentMode != MODE_OBSTACLE) { stopMotors(); return; }
    }

    int leftDistance = scanDirection(ANGLE_LEFT);
    if (currentMode != MODE_OBSTACLE) { stopMotors(); return; }
    int rightDistance = scanDirection(ANGLE_RIGHT);
    if (currentMode != MODE_OBSTACLE) { stopMotors(); return; }

    headServo.write(ANGLE_CENTER);
    yieldDelay(400); 
    if (currentMode != MODE_OBSTACLE) { stopMotors(); return; }

    if (leftDistance == -1 && rightDistance == -1) {
      spinLeft(TURN_SPEED);
      yieldDelay(TURN_DURATION * 1.8); 
    } 
    else if (leftDistance >= rightDistance) {
      spinLeft(TURN_SPEED);
      yieldDelay(TURN_DURATION); 
    } 
    else {
      spinRight(TURN_SPEED);
      yieldDelay(TURN_DURATION); 
    }
    if (currentMode != MODE_OBSTACLE) { stopMotors(); return; }
    stopMotors();
    yieldDelay(400); 
  }
}

// --- Autopilot: Line Following ---
void runLineFollowing() {
  if (currentMode != MODE_LINE) return;
  if (headServo.attached()) headServo.detach();

  int leftIR = digitalRead(pinIR1);
  int rightIR = digitalRead(pinIR2);

  if (leftIR == 0 && rightIR == 0) {
    moveForward(AUTO_DRIVE_SPEED - 10); 
  }
  else if (leftIR == 1 && rightIR == 0) {
    spinLeft(TURN_SPEED);
  }
  else if (leftIR == 0 && rightIR == 1) {
    spinRight(TURN_SPEED);
  }
  else if (leftIR == 1 && rightIR == 1) {
    stopMotors();
  }
  yieldDelay(10); 
  if (currentMode != MODE_LINE) { stopMotors(); return; }
}

// --- Autopilot: Hand Following ---
void runHandFollowing() {
  if (currentMode != MODE_GESTURE) return;
  if (headServo.attached() == false) headServo.attach(pinServo, 500, 2400);

  long distanceCenter = measureDistance();
  if (currentMode != MODE_GESTURE) return;

  if (distanceCenter > 0 && distanceCenter <= DIST_MAX_FOLLOW) {
    absenceCounter = 0;
    
    if (distanceCenter < DIST_TOO_CLOSE) {
      moveBackward(AUTO_DRIVE_SPEED);
    }
    else if (distanceCenter >= DIST_TOO_CLOSE && distanceCenter <= DIST_START_FOLLOW) {
      stopMotors();
    }
    else if (distanceCenter > DIST_START_FOLLOW && distanceCenter <= DIST_MAX_FOLLOW) {
      moveForward(AUTO_DRIVE_SPEED);
    }
    yieldDelay(50); 
    if (currentMode != MODE_GESTURE) { stopMotors(); return; }
  }
  else {
    absenceCounter++;
    stopMotors();

    if (absenceCounter >= ABSENCE_THRESHOLD) {
      headServo.write(ANGLE_LEFT);
      yieldDelay(SERVO_DELAY); 
      if (currentMode != MODE_GESTURE) { stopMotors(); return; }
      long distLeft = measureDistance();

      headServo.write(ANGLE_RIGHT);
      yieldDelay(SERVO_DELAY * 2); 
      if (currentMode != MODE_GESTURE) { stopMotors(); return; }
      long distRight = measureDistance();

      headServo.write(ANGLE_CENTER);
      yieldDelay(SERVO_DELAY); 
      if (currentMode != MODE_GESTURE) { stopMotors(); return; }

      absenceCounter = 0;

      if (distLeft > 0 && distLeft <= DIST_MAX_FOLLOW) {
        spinLeft(TURN_SPEED);
        yieldDelay(PIVOT_TIME); 
        if (currentMode != MODE_GESTURE) { stopMotors(); return; }
        stopMotors();
        yieldDelay(300); 
      }
      else if (distRight > 0 && distRight <= DIST_MAX_FOLLOW) {
        spinRight(TURN_SPEED);
        yieldDelay(PIVOT_TIME); 
        if (currentMode != MODE_GESTURE) { stopMotors(); return; }
        stopMotors();
        yieldDelay(300); 
      }
    }
    else {
      yieldDelay(50); 
      if (currentMode != MODE_GESTURE) { stopMotors(); return; }
    }
  }
}

// --- Main Setup Loop ---
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- Starting Stage 7 Mobile Voice Setup ---");

  activeObstacleThreshold = calculateDynamicThreshold(AUTO_DRIVE_SPEED);

  // Initialize motor outputs
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENB, OUTPUT);
  
  // Sensors
  pinMode(pinIR1, INPUT);
  pinMode(pinIR2, INPUT);
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);

  // Servo init
  ESP32PWM::allocateTimer(0);
  headServo.setPeriodHertz(50);
  headServo.detach();
  stopMotors();

  // --- Network Init: Home Station Mode ---
  WiFi.mode(WIFI_STA);
  WiFi.begin("Edge2", "F@rthebest");
  
  Serial.print("Connecting to Wi-Fi: Edge2");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected successfully!");
  
  // Force Wi-Fi transmitter to maximum output for range stability
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  Serial.print("Use your phone to navigate to: http://");
  Serial.println(WiFi.localIP());

  // --- Web Server Setup ---
  server.on("/", HTTP_GET, []() {
    Serial.println("Serving root dashboard page...");
    server.send(200, "text/html", htmlDashboard);
  });

  // Steering Endpoint Routers
  server.on("/forward", HTTP_GET, []() {
    currentMode = MODE_MANUAL;
    if (headServo.attached()) headServo.detach();
    moveForward(activeDriveSpeed);
    Serial.println("Manual Drive Command: FORWARD");
    server.send(200, "text/plain", "1");
  });
  server.on("/backward", HTTP_GET, []() {
    currentMode = MODE_MANUAL;
    if (headServo.attached()) headServo.detach();
    moveBackward(activeDriveSpeed);
    Serial.println("Manual Drive Command: BACKWARD");
    server.send(200, "text/plain", "1");
  });
  
  // DYNAMIC MANUAL TURNING CONTROL ROUTER
  server.on("/left", HTTP_GET, []() {
    currentMode = MODE_MANUAL;
    if (headServo.attached()) headServo.detach();
    
    // Apply dynamic speed + 25-point torque boost for skid steering (capped at max 220)
    int manualTurnSpeed = activeDriveSpeed + 25;
    if (manualTurnSpeed > 220) manualTurnSpeed = 220;
    
    spinLeft(manualTurnSpeed);
    Serial.print("Manual Drive Command: SPIN LEFT (Speed: ");
    Serial.print(manualTurnSpeed);
    Serial.println(")");
    server.send(200, "text/plain", "1");
  });
  
  server.on("/right", HTTP_GET, []() {
    currentMode = MODE_MANUAL; 
    if (headServo.attached()) headServo.detach();
    
    // Apply dynamic speed + 25-point torque boost for skid steering (capped at max 220)
    int manualTurnSpeed = activeDriveSpeed + 25;
    if (manualTurnSpeed > 220) manualTurnSpeed = 220;
    
    spinRight(manualTurnSpeed);
    Serial.print("Manual Drive Command: SPIN RIGHT (Speed: ");
    Serial.print(manualTurnSpeed);
    Serial.println(")");
    server.send(200, "text/plain", "1");
  });
  
  server.on("/stop", HTTP_GET, []() {
    currentMode = MODE_MANUAL;
    stopMotors();
    Serial.println("Manual Drive Command: STOP");
    server.send(200, "text/plain", "1");
  });

  server.on("/set_speed", HTTP_GET, []() {
    if (server.hasArg("val")) {
      activeDriveSpeed = server.arg("val").toInt();
      Serial.print("Manual speed updated to: ");
      Serial.println(activeDriveSpeed);
    }
    server.send(200, "text/plain", "1");
  });

  server.on("/drift", HTTP_GET, []() {
    currentMode = MODE_MANUAL;
    if (headServo.attached()) headServo.detach();
    server.send(200, "text/plain", "1");
    executeDriftManeuver();
  });

  // Autopilot Toggles
  server.on("/mode_manual", HTTP_GET, []() {
    currentMode = MODE_MANUAL;
    stopMotors();
    if (headServo.attached()) headServo.detach();
    Serial.println("WEB COMMAND: MODE changed to MANUAL steering");
    server.send(200, "text/plain", "Manual Mode");
  });
  server.on("/mode_obstacle", HTTP_GET, []() {
    currentMode = MODE_OBSTACLE;
    stopMotors();
    Serial.println("WEB COMMAND: MODE changed to OBSTACLE AVOIDANCE");
    server.send(200, "text/plain", "Obstacle Mode");
  });
  server.on("/mode_line", HTTP_GET, []() {
    currentMode = MODE_LINE;
    stopMotors();
    if (headServo.attached()) headServo.detach();
    Serial.println("WEB COMMAND: MODE changed to LINE FOLLOWING");
    server.send(200, "text/plain", "Line Mode");
  });
  server.on("/mode_gesture", HTTP_GET, []() {
    currentMode = MODE_GESTURE;
    stopMotors();
    Serial.println("WEB COMMAND: MODE changed to HAND FOLLOWING");
    server.send(200, "text/plain", "Gesture Mode");
  });

  // Simple 404 handler (Saves memory over 302 redirects)
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not Found");
  });

  server.begin();
  Serial.println("Web Server Online.");
}

void loop() {
  server.handleClient();

  switch (currentMode) {
    case MODE_MANUAL:
      break;
    case MODE_OBSTACLE:
      runObstacleAvoidance();
      break;
    case MODE_LINE:
      runLineFollowing();
      break;
    case MODE_GESTURE:
      runHandFollowing();
      break;
  }
}