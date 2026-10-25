#include "esp_camera.h"
#include "esp_system.h"
#include <WiFi.h>
#include <WebServer.h>

// ================= USER SETTINGS =================
const char* AP_SSID = "ESP32-Camera";
const char* AP_PASS = "12345678";          // at least 8 characters

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
#define PIN_SDA    4
#define PIN_SCL    5
#define PIN_PWDN   -1
#define PIN_RESET  -1

#define XCLK_HZ  8000000   // start here; raise to 6, 8, 10 MHz and stop before the image tears
// =================================================

#define OUT_W 160          // size sent to the browser (capture is 320x240, shrunk 2x)
#define OUT_H 120

WebServer server(80);
static uint16_t *smallBuf = nullptr;   // holds the 160x120 output frame

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

function draw(img, frame, swap) {
  const d = img.data;
  for (let i = 0, j = 0; i < frame.length; i += 2, j += 4) {
    const v = swap ? ((frame[i + 1] << 8) | frame[i]) : ((frame[i] << 8) | frame[i + 1]);
    const r5 = (v >> 11) & 31, g6 = (v >> 5) & 63, b5 = v & 31;
    d[j]     = (r5 << 3) | (r5 >> 2);
    d[j + 1] = (g6 << 2) | (g6 >> 4);
    d[j + 2] = (b5 << 3) | (b5 >> 2);
    d[j + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

async function run() {
  try {
    const res = await fetch('/stream', {cache: 'no-store'});
    const reader = res.body.getReader();
    const hdr = new Uint8Array(4);
    let hfill = 0, W = 0, H = 0, frameLen = 0, frame = null, fill = 0, img = null;
    let frames = 0, t0 = performance.now();

    while (true) {
      const {value, done} = await reader.read();
      if (done) break;
      let p = 0;
      while (p < value.length) {
        if (!W) {                                   // first 4 bytes: width and height
          const n = Math.min(4 - hfill, value.length - p);
          hdr.set(value.subarray(p, p + n), hfill);
          hfill += n; p += n;
          if (hfill === 4) {
            W = (hdr[0] << 8) | hdr[1];
            H = (hdr[2] << 8) | hdr[3];
            frameLen = W * H * 2;
            frame = new Uint8Array(frameLen);
            cv.width = W; cv.height = H;
            img = ctx.createImageData(W, H);
          }
        } else {                                    // then raw frames back-to-back
          const n = Math.min(frameLen - fill, value.length - p);
          frame.set(value.subarray(p, p + n), fill);
          fill += n; p += n;
          if (fill === frameLen) {
            draw(img, frame, swapBox.checked);
            fill = 0; frames++;
            const now = performance.now();
            if (now - t0 > 1000) {
              fpsEl.textContent = (frames * 1000 / (now - t0)).toFixed(1) + ' fps  (' + W + 'x' + H + ')';
              frames = 0; t0 = now;
            }
          }
        }
      }
    }
  } catch (e) {}
  fpsEl.textContent = 'stream lost, reconnecting...';
  setTimeout(run, 1000);
}
run();
</script>
</body></html>
)rawliteral";

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

static bool sendAll(WiFiClient &c, const uint8_t *p, size_t n) {
  uint32_t t = millis();
  while (n) {
    size_t w = c.write(p, n);
    if (w == 0) {
      if (!c.connected() || millis() - t > 2000) return false;
      delay(1);
      continue;
    }
    p += w; n -= w; t = millis();
  }
  return true;
}

// Keep every second pixel of every second row: 320x240 -> 160x120
static void downscale2x(const camera_fb_t *fb) {
  const uint16_t *src = (const uint16_t *)fb->buf;
  size_t w = fb->width, h = fb->height;
  uint16_t *dst = smallBuf;
  for (size_t y = 0; y + 1 < h && (y / 2) < OUT_H; y += 2) {
    const uint16_t *row = src + y * w;
    for (size_t x = 0; x + 1 < w && (x / 2) < OUT_W; x += 2) *dst++ = row[x];
  }
}

// Streams frames continuously over one connection (one viewer at a time)
void handleStream() {
  WiFiClient client = server.client();
  client.setNoDelay(true);
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: application/octet-stream\r\n"
               "Cache-Control: no-store\r\n"
               "Connection: close\r\n\r\n");

  bool hdrSent = false;
  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { delay(5); continue; }

    if (fb->width != OUT_W * 2 || fb->height != OUT_H * 2) {   // unexpected size, skip it
      esp_camera_fb_return(fb);
      delay(5);
      continue;
    }

    downscale2x(fb);
    esp_camera_fb_return(fb);              // give the camera frame back before sending

    if (!hdrSent) {
      uint8_t h[4] = { (uint8_t)(OUT_W >> 8), (uint8_t)OUT_W,
                       (uint8_t)(OUT_H >> 8), (uint8_t)OUT_H };
      if (!sendAll(client, h, 4)) break;
      hdrSent = true;
    }
    if (!sendAll(client, (const uint8_t *)smallBuf, (size_t)OUT_W * OUT_H * 2)) break;
  }
  client.stop();
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
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QVGA;             // 320x240, works on your sensor
  config.grab_mode    = CAMERA_GRAB_LATEST;     // never capture while data is being sent
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
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
  delay(3000);   // gives you time to open the serial monitor
  Serial.printf("Reset reason: %d  (1=power-on, 3=software, 4=crash, 5-7=watchdog, 9=brownout)\n",
                (int)esp_reset_reason());

  // 1. WiFi first: the ESP32 creates its own network
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  // 2. Output buffer, allocated once
  smallBuf = (uint16_t *)heap_caps_malloc(OUT_W * OUT_H * 2, MALLOC_CAP_8BIT);
  if (!smallBuf) {
    Serial.println("Failed to allocate small frame buffer.");
    return;
  }

  // 3. Camera
  if (!startCamera()) {
    Serial.println("Camera failed. Check wiring, pull-ups, PWDN/RESET and power.");
    return;
  }

  // 4. Web server
  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.begin();
  Serial.print("Join WiFi '");
  Serial.print(AP_SSID);
  Serial.print("' and open http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();
}