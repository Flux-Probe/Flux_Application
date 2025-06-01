/*
 * ESP32 Access Point + Web-Based PWM Motor Control + AS5600 Angle Display and Plot
 * ----------------------------------------------------------------------------------
 * ✅ No router needed – ESP32 creates its own Wi-Fi
 * ✅ Use a slider on a web page to control motor speed
 * ✅ Shows live angle from AS5600 magnetic encoder in browser
 * ✅ Real-time plot of angle below the slider
 * 
 * 🛠️ Wiring Instructions:
 * 
 * 🔧 Motor Control (S8050 NPN):
 * - ESP32 GPIO 23 → 1kΩ resistor → Base (B) of S8050
 * - S8050 Collector (C) → Motor –
 * - Motor + → 3.7V battery +
 * - S8050 Emitter (E) → Battery – → ESP32 GND
 * - Flyback diode across motor: banded end to Motor +, other end to Motor –
 * 
 * 🧭 AS5600 Encoder (I2C):
 * - VCC → ESP32 3.3V
 * - GND → ESP32 GND
 * - SDA → ESP32 GPIO 21
 * - SCL → ESP32 GPIO 22
 * 
 * 📱 Connect to Wi-Fi: FluxControl (password: flux1234)
 * 🌐 Open in browser: http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>

const char* ssid = "FluxControl";
const char* password = "flux1234";

WebServer server(80);

// Motor control
const int motorPin = 23;
const int pwmFreq = 5000;
const int pwmRes = 8;

// AS5600 I2C
#define AS5600_ADDR 0x36
#define ANGLE_MSB 0x0E
#define ANGLE_LSB 0x0F

int readAS5600Angle() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(ANGLE_MSB);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    int msb = Wire.read();
    int lsb = Wire.read();
    return (msb << 8) | lsb;
  }
  return -1;
}

void handleRoot() {
  int angle = readAS5600Angle();
  float degrees = (angle >= 0) ? (angle * 360.0 / 4096.0) : -1.0;

  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>Motor + Encoder</title>
      <style>
        body { font-family: sans-serif; text-align: center; margin-top: 30px; }
        input[type=range] { width: 80%; }
        canvas { border: 1px solid #ccc; margin-top: 20px; }
      </style>
    </head>
    <body>
      <h2>Motor Speed Control</h2>
      <input type="range" min="0" max="255" value="0" id="slider" oninput="updateSpeed(this.value)">
      <p>Speed: <span id="value">0</span></p>
      <h3>AS5600 Angle: <span id="angle">)rawliteral";

  html += String(degrees, 1);
  html += R"rawliteral(</span>°</h3>

      <canvas id="plot" width="300" height="150"></canvas>

      <script>
        let canvas = document.getElementById("plot");
        let ctx = canvas.getContext("2d");
        let data = [];

        function updateSpeed(val) {
          document.getElementById("value").innerText = val;
          fetch(`/set?speed=${val}`);
        }

        function drawPlot() {
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.beginPath();
          ctx.moveTo(0, canvas.height - data[0]);
          for (let i = 1; i < data.length; i++) {
            ctx.lineTo(i, canvas.height - data[i]);
          }
          ctx.strokeStyle = "blue";
          ctx.stroke();
        }

        setInterval(() => {
          fetch('/angle')
            .then(res => res.text())
            .then(deg => {
              let angle = parseFloat(deg);
              document.getElementById('angle').innerText = angle.toFixed(1);

              let scaled = Math.min(canvas.height, Math.max(0, angle * canvas.height / 360));
              if (data.length >= canvas.width) data.shift();
              data.push(scaled);
              drawPlot();
            });
        }, 200);
      </script>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleSetSpeed() {
  if (server.hasArg("speed")) {
    int speed = server.arg("speed").toInt();
    speed = constrain(speed, 0, 255);
    ledcWrite(motorPin, speed);
    Serial.printf("Set speed to %d\n", speed);
  }
  server.send(204);
}

void handleAngle() {
  int angle = readAS5600Angle();
  float degrees = (angle >= 0) ? (angle * 360.0 / 4096.0) : -1.0;
  server.send(200, "text/plain", String(degrees));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  ledcAttach(motorPin, pwmFreq, pwmRes);
  ledcWrite(motorPin, 0);

  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.print("Wi-Fi Name: "); Serial.println(ssid);
  Serial.println("Open browser to: http://192.168.4.1");

  server.on("/", handleRoot);
  server.on("/set", handleSetSpeed);
  server.on("/angle", handleAngle);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
}
