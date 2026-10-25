#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>

// ================= USER SETTINGS =================
const char* WIFI_SSID = "TAMU_IoT";   // your network name (2.4 GHz only)
const char* WIFI_PASS = "";           // leave empty for an open network

// OV7670 -> ESP32-S3 pins (change to match your wiring)
#define PIN_D0     8
#define PIN_D1     9
#define PIN_D2     10
#define PIN_D3     11
#define PIN_D4     12
#define PIN_D5     13
#define PIN_D6     14
#define PIN_D7     15
#define PIN_XCLK   20
#define PIN_PCLK   47
#define PIN_VSYNC  6
#define PIN_HREF   7
#define PIN_SDA    4    // SIOD
#define PIN_SCL    5    // SIOC
#define PIN_PWDN   -1   // tie the module's PWDN pin to GND if not driven
#define PIN_RESET  -1   // tie the module's RESET pin to 3.3V if not driven

#define XCLK_HZ    8000000   // OV7670 likes 10-20 MHz; lower to 8 MHz if the image is noisy
// =================================================

WebServer server(80);

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:Helvetica;text-align:center;margin:12px}
  canvas{width:100%;max-width:640px;image-rendering:pixelated;background:#000}
  #fps{color:#666;font-size:14px}
</style>
</head><body>
<h2>OV7670 Live Feed</h2>
<canvas id="c" width="160" height="120"></canvas>
<p><label><input type="checkbox" id="swap"> swap bytes (tick if colors look wrong)</label></p>
<p id="fps">connecting...</p>
<script>
const cv = document.getElementById('c');
const ctx = cv.getContext('2d');
const swapBox = document.getElementById('swap');
const fpsEl = document.getElementById('fps');
let img = null, frames = 0, t0 = performance.now();

async function loop() {
  try {
    const r = await fetch('/raw?t=' + Date.now(), {cache: 'no-store'});
    if (!r.ok) throw new Error('bad status');
    const W = parseInt(r.headers.get('X-W'));
    const H = parseInt(r.headers.get('X-H'));
    const b = new Uint8Array(await r.arrayBuffer());
    if (W && H && b.length === W * H * 2) {
      if (cv.width !== W || cv.height !== H || !img) {
        cv.width = W; cv.height = H;
        img = ctx.createImageData(W, H);
      }
      const d = img.data, swap = swapBox.checked;
      for (let i = 0, j = 0; i < b.length; i += 2, j += 4) {
        const v = swap ? ((b[i + 1] << 8) | b[i]) : ((b[i] << 8) | b[i + 1]);
        const r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
        d[j]     = (r5 << 3) | (r5 >> 2);
        d[j + 1] = (g6 << 2) | (g6 >> 4);
        d[j + 2] = (b5 << 3) | (b5 >> 2);
        d[j + 3] = 255;
      }
      ctx.putImageData(img, 0, 0);
      frames++;
      const now = performance.now();
      if (now - t0 > 1000) {
        fpsEl.textContent = (frames * 1000 / (now - t0)).toFixed(1) + ' fps  (' + W + 'x' + H + ')';
        frames = 0; t0 = now;
      }
    }
    setTimeout(loop, 10);
  } catch (e) {
    fpsEl.textContent = 'no frame, retrying...';
    setTimeout(loop, 1000);
  }
}
loop();
</script>
</body></html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleRaw() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "capture failed");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-W", String(fb->width));
  server.sendHeader("X-H", String(fb->height));
  // send_P blocks until the data is sent, so the frame stays valid until we return it
  server.send_P(200, "application/octet-stream", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

bool startCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = PIN_D0;
  config.pin_d1 = PIN_D1;
  config.pin_d2 = PIN_D2;
  config.pin_d3 = PIN_D3;
  config.pin_d4 = PIN_D4;
  config.pin_d5 = PIN_D5;
  config.pin_d6 = PIN_D6;
  config.pin_d7 = PIN_D7;
  config.pin_xclk  = PIN_XCLK;
  config.pin_pclk  = PIN_PCLK;
  config.pin_vsync = PIN_VSYNC;
  config.pin_href  = PIN_HREF;
  config.pin_sccb_sda = PIN_SDA;
  config.pin_sccb_scl = PIN_SCL;
  config.pin_pwdn  = PIN_PWDN;
  config.pin_reset = PIN_RESET;

  config.xclk_freq_hz = XCLK_HZ;
  config.pixel_format = PIXFORMAT_RGB565;   // OV7670 cannot output JPEG
  config.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size  = FRAMESIZE_QVGA;    // 320x240
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.fb_count    = 2;
  } else {
    config.frame_size  = FRAMESIZE_QQVGA;   // 160x120, small enough for internal RAM
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.fb_count    = 1;
  }
  Serial.printf("PSRAM: %s\n", psramFound() ? "found" : "NOT found");

  esp_err_t err = esp_camera_init(&config);
  Serial.printf("Camera init: 0x%x (%s)\n", err, esp_err_to_name(err));
  if (err != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("Sensor PID: 0x%x (OV7670 = 0x76)\n", s->id.PID);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // // 1. WiFi first, while internal RAM is still free
  // WiFi.mode(WIFI_STA);
  // Serial.printf("Connecting to %s", WIFI_SSID);
  // if (strlen(WIFI_PASS) == 0) WiFi.begin(WIFI_SSID);
  // else WiFi.begin(WIFI_SSID, WIFI_PASS);

  // int tries = 0;
  // while (WiFi.status() != WL_CONNECTED && tries < 60) {
  //   delay(500);
  //   Serial.print(".");
  //   tries++;
  // }
  // if (WiFi.status() != WL_CONNECTED) {
  //   Serial.println("\nWiFi failed. Check the name, the 2.4 GHz band, and any device registration.");
  //   return;
  // }
  // WiFi.setSleep(false);
  // Serial.println("\nWiFi connected.");
  // 1. WiFi first: the ESP32 creates its own network
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-Camera", "12345678");   // password must be at least 8 characters
  Serial.print("Connect to WiFi 'ESP32-Camera', then open http://");
  Serial.println(WiFi.softAPIP());           // normally 192.168.4.1

  // 2. Camera
  if (!startCamera()) {
    Serial.println("Camera failed. Check wiring, pull-ups, PWDN/RESET and power.");
    return;
  }

  // 3. Web server
  server.on("/", handleRoot);
  server.on("/raw", handleRaw);
  server.begin();
  Serial.print("Open in a browser: http://");
  Serial.println(WiFi.softAPIP());
//   // 3. Web server
//   server.on("/", handleRoot);
//   server.on("/raw", handleRaw);
//   server.begin();
//   Serial.print("Open in a browser: http://");
//   Serial.println(WiFi.localIP());
}

void loop() {
  server.handleClient();
}