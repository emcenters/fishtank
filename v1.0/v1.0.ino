#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "driver/ledc.h"

// ================= USER SETTINGS =================
const char* AP_SSID = "ESP32-Camera";
const char* AP_PASS = "12345678";          // at least 8 characters

// OV7670 -> ESP32-S3 pins
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

#define PIN_SERVO  18

#define XCLK_HZ    10000000   // if the image tears or the probe fails, try 8000000 or 20000000

// Servo pulse range (500-2400 us suits most micro servos)
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400

// Detection tuning
#define DIFF_THRESH    25     // how much a pixel's brightness must change (0-255)
#define MIN_PIXELS     80     // ignore changes smaller than this many pixels (noise)
#define GLOBAL_FRAC    0.40f  // if more than 40% of the image changes, treat it as a lighting change

// Servo aiming
#define CAM_FOV_DEG    45.0f  // camera's horizontal field of view; tune by testing
#define SERVO_DIR      1      // set to -1 if the servo turns the wrong way
#define MAX_STEP_DEG   5.0f   // max servo movement per frame (smoothness)
#define DEADBAND_DEG   2.0f   // ignore changes smaller than this (stops twitching)
// =================================================

#define OUT_W 160
#define OUT_H 120
#define NPIX  (OUT_W * OUT_H)

// Overlay colors in the camera's byte order (shown correctly with "swap bytes" unticked)
#define COL_GREEN 0xE007
#define COL_RED   0x00F8

WebServer server(80);

static uint16_t *workBuf  = nullptr;   // frame the vision task draws on
static uint16_t *shareBuf = nullptr;   // latest finished frame
static uint16_t *sendBuf  = nullptr;   // copy used by the stream
static uint8_t  *bg       = nullptr;   // background brightness model
static SemaphoreHandle_t frameMutex;
static volatile uint32_t frameId = 0;

struct Det { bool found; float cx, cy, sx, sy; };

// ---------------- Servo (LEDC timer 1 / channel 1; the camera uses timer 0 / channel 0) ----------------
void servoInit() {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_LOW_SPEED_MODE;
  t.duty_resolution = LEDC_TIMER_14_BIT;
  t.timer_num       = LEDC_TIMER_1;
  t.freq_hz         = 50;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&t);

  ledc_channel_config_t c = {};
  c.gpio_num   = PIN_SERVO;
  c.speed_mode = LEDC_LOW_SPEED_MODE;
  c.channel    = LEDC_CHANNEL_1;
  c.timer_sel  = LEDC_TIMER_1;
  c.duty       = 0;
  c.hpoint     = 0;
  ledc_channel_config(&c);
}

void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t us   = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
  uint32_t duty = us * 16384UL / 20000UL;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// ---------------- Image helpers ----------------
// 320x240 -> 160x120 by keeping every second pixel of every second row
static void downscale2x(const camera_fb_t *fb, uint16_t *dst) {
  const uint16_t *src = (const uint16_t *)fb->buf;
  size_t w = fb->width, h = fb->height;
  for (size_t y = 0; y + 1 < h && (y / 2) < OUT_H; y += 2) {
    const uint16_t *row = src + y * w;
    for (size_t x = 0; x + 1 < w && (x / 2) < OUT_W; x += 2) *dst++ = row[x];
  }
}

// Brightness of one pixel (the camera's bytes arrive swapped when read as uint16_t)
static inline uint8_t grayOf(uint16_t p) {
  uint16_t v = (uint16_t)((p << 8) | (p >> 8));
  uint8_t r = ((v >> 11) & 31) << 3;
  uint8_t g = ((v >> 5)  & 63) << 2;
  uint8_t b = (v & 31) << 3;
  return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

static inline void putPx(int x, int y, uint16_t col) {
  if (x >= 0 && x < OUT_W && y >= 0 && y < OUT_H) workBuf[y * OUT_W + x] = col;
}

static void drawBox(int x0, int y0, int x1, int y1, uint16_t col) {
  for (int x = x0; x <= x1; x++) { putPx(x, y0, col); putPx(x, y1, col); }
  for (int y = y0; y <= y1; y++) { putPx(x0, y, col); putPx(x1, y, col); }
}


// Compare workBuf with the background model and find where it changed
static Det detect(bool &globalChange) {
  Det d = { false, 0, 0, 0, 0 };
  globalChange = false;
  uint32_t count = 0;
  float sumx = 0, sumy = 0, sumx2 = 0, sumy2 = 0;

  for (int y = 0; y < OUT_H; y++) {
    for (int x = 0; x < OUT_W; x++) {
      int i = y * OUT_W + x;
      int g = grayOf(workBuf[i]);
      int diff = abs(g - (int)bg[i]);
      if (diff > DIFF_THRESH) {
        count++;
        sumx += x;  sumy += y;
        sumx2 += (float)x * x;  sumy2 += (float)y * y;
      }
      bg[i] = (uint8_t)((int)bg[i] + (g - (int)bg[i]) / 8);   // slowly follow the scene
    }
  }

  if (count > GLOBAL_FRAC * NPIX) {            // whole image changed: lighting or auto-exposure
    globalChange = true;
    for (int i = 0; i < NPIX; i++) bg[i] = grayOf(workBuf[i]);
    return d;
  }
  if (count >= MIN_PIXELS) {
    d.found = true;
    d.cx = sumx / count;
    d.cy = sumy / count;
    float vx = sumx2 / count - d.cx * d.cx;
    float vy = sumy2 / count - d.cy * d.cy;
    d.sx = sqrtf(vx > 0 ? vx : 0);
    d.sy = sqrtf(vy > 0 ? vy : 0);
  }
  return d;
}

// ---------------- Vision task: camera, detection, servo ----------------
void visionTask(void *) {
  float servoAngle = 90, filtX = -1;
  int warmup = 0, lostFrames = 0;
  uint32_t lastPrint = 0;
  servoWrite(90);

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    if (fb->width != OUT_W * 2 || fb->height != OUT_H * 2) {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    downscale2x(fb, workBuf);
    esp_camera_fb_return(fb);

    // The first few frames just build the background
    if (warmup < 10) {
      for (int i = 0; i < NPIX; i++) bg[i] = grayOf(workBuf[i]);
      warmup++;
    } else {
      bool globalChange;
      Det d = detect(globalChange);

      if (d.found) {
        lostFrames = 0;
        filtX = (filtX < 0) ? d.cx : 0.6f * filtX + 0.4f * d.cx;

        // Turn the servo toward the object, smoothly
        float target = 90.0f + SERVO_DIR * ((filtX - OUT_W / 2.0f) / OUT_W) * CAM_FOV_DEG;
        target = constrain(target, 0.0f, 180.0f);
        float diff = target - servoAngle;
        if (fabsf(diff) > DEADBAND_DEG) {
          servoAngle += constrain(diff, -MAX_STEP_DEG, MAX_STEP_DEG);
          servoWrite((int)servoAngle);
        }

        // Draw the box and a crosshair on the frame
        int x0 = (int)(d.cx - 2 * d.sx), x1 = (int)(d.cx + 2 * d.sx);
        int y0 = (int)(d.cy - 2 * d.sy), y1 = (int)(d.cy + 2 * d.sy);
        drawBox(constrain(x0, 0, OUT_W - 1), constrain(y0, 0, OUT_H - 1),
                constrain(x1, 0, OUT_W - 1), constrain(y1, 0, OUT_H - 1), COL_GREEN);
        for (int k = -4; k <= 4; k++) {
          putPx((int)d.cx + k, (int)d.cy, COL_RED);
          putPx((int)d.cx, (int)d.cy + k, COL_RED);
        }

        if (millis() - lastPrint > 500) {
          lastPrint = millis();
          Serial.printf("Object at x=%.0f y=%.0f (size %d px)  servo=%d deg\n",
                        d.cx, d.cy, (int)(4 * d.sx * d.sy), (int)servoAngle);
        }
      } else if (++lostFrames > 30) {
        filtX = -1;
      }
    }

    // Hand the finished frame to the stream
    xSemaphoreTake(frameMutex, portMAX_DELAY);
    memcpy(shareBuf, workBuf, NPIX * 2);
    frameId++;
    xSemaphoreGive(frameMutex);
  }
}

// ---------------- Web page and stream ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:Helvetica;text-align:center;margin:12px}
  canvas{width:100%;max-width:640px;image-rendering:pixelated;background:#000}
  #fps{color:#666;font-size:14px}
</style>
</head><body>
<h2>OV7670 Object Tracker</h2>
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
        if (!W) {
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
        } else {
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

void handleStream() {
  WiFiClient client = server.client();
  client.setNoDelay(true);
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: application/octet-stream\r\n"
               "Cache-Control: no-store\r\n"
               "Connection: close\r\n\r\n");

  uint8_t h[4] = { (uint8_t)(OUT_W >> 8), (uint8_t)OUT_W,
                   (uint8_t)(OUT_H >> 8), (uint8_t)OUT_H };
  if (!sendAll(client, h, 4)) { client.stop(); return; }

  uint32_t lastId = 0;
  while (client.connected()) {
    if (frameId == lastId) { delay(2); continue; }   // wait for a new frame

    xSemaphoreTake(frameMutex, portMAX_DELAY);
    memcpy(sendBuf, shareBuf, NPIX * 2);
    lastId = frameId;
    xSemaphoreGive(frameMutex);

    if (!sendAll(client, (const uint8_t *)sendBuf, NPIX * 2)) break;
  }
  client.stop();
}

// ---------------- Camera setup ----------------
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
  config.frame_size   = FRAMESIZE_QVGA;      // 320x240, shrunk to 160x120 for processing
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  Serial.printf("Camera init: 0x%x (%s)\n", err, esp_err_to_name(err));
  if (err != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("Sensor PID: 0x%x (OV7670 = 0x76)\n", s->id.PID);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.printf("PSRAM: %s\n", psramFound() ? "found" : "NOT found");

  workBuf  = (uint16_t *)heap_caps_malloc(NPIX * 2, MALLOC_CAP_8BIT);
  shareBuf = (uint16_t *)heap_caps_malloc(NPIX * 2, MALLOC_CAP_8BIT);
  sendBuf  = (uint16_t *)heap_caps_malloc(NPIX * 2, MALLOC_CAP_8BIT);
  bg       = (uint8_t  *)heap_caps_malloc(NPIX,     MALLOC_CAP_8BIT);
  if (!workBuf || !shareBuf || !sendBuf || !bg) {
    Serial.println("Buffer allocation failed.");
    return;
  }
  memset(workBuf, 0, NPIX * 2);
  memset(shareBuf, 0, NPIX * 2);
  memset(bg, 0, NPIX);
  frameMutex = xSemaphoreCreateMutex();

  if (!startCamera()) {
    Serial.println("Camera failed. Check wiring, pull-ups, PWDN/RESET and power.");
    return;
  }

  servoInit();
  xTaskCreatePinnedToCore(visionTask, "vision", 8192, nullptr, 2, nullptr, 1);

  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.softAP(AP_SSID, AP_PASS);
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
  delay(1);
}