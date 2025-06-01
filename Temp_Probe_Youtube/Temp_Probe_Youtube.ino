#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

const char* ssid = "FluxControl";
const char* password = "flux1234";

WebServer server(80);
const int tempPin = 34;
const float vRef = 3.3;
const int adcMax = 4095;

// Thermistor setup
const float R_FIXED = 10000.0;  // Your actual fixed resistor

// Steinhart-Hart coefficients from GitHub code
const float A = 0.0007876984931;
const float B = 0.0002069666861;
const float C = 0.0000001202652917;

// Moving average buffer
const int bufferSize = 40;
float tempBuffer[bufferSize];
int bufferIndex = 0;
int bufferCount = 0;

float readTemperatureC() {
  int raw = analogRead(tempPin);
  float voltage = (raw / (float)adcMax) * vRef;
  if (voltage <= 0 || voltage >= vRef) return -1000;

  // Corrected for thermistor in top leg
  float resistance = ((vRef - voltage) / voltage) * R_FIXED;
  float logR = log(resistance);
  float invT = A + B * logR + C * pow(logR, 3);
  float tempC = (1.0 / invT) - 273.15;

  return tempC;
}

float readSmoothedTemperature() {
  float raw = readTemperatureC();
  tempBuffer[bufferIndex] = raw;
  bufferIndex = (bufferIndex + 1) % bufferSize;
  if (bufferCount < bufferSize) bufferCount++;

  float sum = 0;
  for (int i = 0; i < bufferCount; i++) {
    sum += tempBuffer[i];
  }
  return sum / bufferCount;
}

void handleTemp() {
  float raw = readTemperatureC();
  float smoothed = readSmoothedTemperature();
  String payload = String(raw, 2) + "," + String(smoothed, 2);
  server.send(200, "text/plain", payload);
}

void handleRoot() {
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>Temperature Monitor</title>
      <style>
        body { font-family: sans-serif; text-align: center; padding-top: 40px; }
        h1 { font-size: 2.5em; }
        #temp { font-size: 1.5em; color: #2196F3; margin-top: 20px; }
        canvas { border: 1px solid #ccc; margin-top: 20px; }
        #plotLabels {
          position: absolute;
          left: -50px;
          top: 0;
          height: 150px;
          display: flex;
          flex-direction: column;
          justify-content: space-between;
        }
        #plotWrapper {
          display: inline-block;
          position: relative;
        }
      </style>
    </head>
    <body>
      <h1>Live Temperature</h1>
      <div id="temp">Loading...</div>
      <div id="plotWrapper">
        <canvas id="plot" width="300" height="150"></canvas>
        <div id="plotLabels">
          <span id="tempMaxLabel" style="font-size: 0.9em;">Max</span>
          <span id="tempMinLabel" style="font-size: 0.9em;">Min</span>
        </div>
      </div>

      <script>
        let canvas = document.getElementById("plot");
        let ctx = canvas.getContext("2d");
        let rawData = [], smoothData = [];

        function updateTemp() {
          fetch('/temp')
            .then(res => res.text())
            .then(val => {
              let parts = val.split(',');
              let raw = parseFloat(parts[0]);
              let smooth = parseFloat(parts[1]);
              document.getElementById('temp').innerHTML = smooth.toFixed(2) + " &deg;C";

              if (rawData.length >= canvas.width) rawData.shift();
              if (smoothData.length >= canvas.width) smoothData.shift();

              rawData.push(raw);
              smoothData.push(smooth);

              drawPlot();
            });
        }

        function drawPlot() {
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          if (rawData.length === 0) return;

          let combined = rawData.concat(smoothData);
          let minTemp = Math.min(...combined);
          let maxTemp = Math.max(...combined);
          let range = maxTemp - minTemp;
          if (range < 5) range = 5;

          document.getElementById("tempMinLabel").innerHTML = minTemp.toFixed(1) + " &deg;C";
          document.getElementById("tempMaxLabel").innerHTML = maxTemp.toFixed(1) + " &deg;C";

          ctx.beginPath();
          for (let i = 0; i < rawData.length; i++) {
            let norm = (rawData[i] - minTemp) / range;
            let y = canvas.height - (norm * canvas.height);
            if (i === 0) ctx.moveTo(i, y);
            else ctx.lineTo(i, y);
          }
          ctx.strokeStyle = "blue";
          ctx.stroke();

          ctx.beginPath();
          for (let i = 0; i < smoothData.length; i++) {
            let norm = (smoothData[i] - minTemp) / range;
            let y = canvas.height - (norm * canvas.height);
            if (i === 0) ctx.moveTo(i, y);
            else ctx.lineTo(i, y);
          }
          ctx.strokeStyle = "red";
          ctx.stroke();
        }

        setInterval(updateTemp, 100);
        updateTemp();
      </script>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(tempPin, INPUT);

  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");
  Serial.println("Wi-Fi: FluxControl");
  Serial.println("Go to: http://192.168.4.1");

  server.on("/", handleRoot);
  server.on("/temp", handleTemp);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  server.handleClient();


  Serial.print("Raw ADC: ");
  Serial.print(analogRead(tempPin));
  Serial.print(" | Voltage: ");
  float voltage = (analogRead(tempPin) / 4095.0) * 3.3;
  Serial.println(voltage);

}
