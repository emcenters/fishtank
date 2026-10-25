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
#define PIN_XCLK   16
#define PIN_PCLK   17
#define PIN_VSYNC  6
#define PIN_HREF   7
#define PIN_SDA    4
#define PIN_SCL    5
#define PIN_PWDN   -1
#define PIN_RESET  -1

#define PIN_SERVO  18

#define XCLK_HZ    10000000

// Servo pulse range (500-2400 us suits most micro servos)
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400

// ---- Detection tuning (the image is analyzed at 320x240) ----
#define DIFF_THRESH       18     // brightness change a pixel needs to count as changed (lower = more sensitive)
#define CELL_MIN_PIXELS   6      // changed pixels needed in an 8x8 cell for the cell to be "active"
#define MIN_TOTAL_PIXELS  14     // changed pixels needed in a whole group to count as an object
#define GLOBAL_FRAC       0.40f  // if more than 40% of the image changes, treat it as a lighting change

// ---- Servo aiming ----
#define CAM_FOV_DEG    45.0f  // camera's horizontal field of view; tune by testing
#define SERVO_DIR      1      // set to -1 if the servo turns the wrong way
#define MAX_STEP_DEG   5.0f   // max servo movement per frame
#define DEADBAND_DEG   2.0f   // ignore smaller changes (stops twitching)
// =================================================

#define OUT_W 160             // size sent to the browser
#define OUT_H 120
#define NPIX  (OUT_W * OUT_H)

#define DET_W 320             // size used for detection
#define DET_H 240
#define CELL  8
#define GRID_W (DET_W / CELL)
#define GRID_H (DET_H / CELL)

// Overlay colors in the camera's byte order (correct with "swap bytes" unticked)
#define COL_GREEN 0xE007
#define COL_RED   0x00F8

// Must be defined before any function that uses it (Arduino auto-generates prototypes)
struct Det { bool found; float cx, cy, sx, sy; uint32_t n; };

WebServer server(80);

static uint16_t *workBuf  = nullptr;
static uint16_t *shareBuf = nullptr;
static uint16_t *sendBuf  = nullptr;
static uint8_t  *bg       = nullptr;      // background brightness, 320x240
static SemaphoreHandle_t frameMutex;
static volatile uint32_t frameId = 0;

static uint8_t  cellCnt[GRID_H][GRID_W];
static uint8_t  cellMark[GRID_H][GRID_W];
static uint16_t cellStack[GRID_W * GRID_H];

static bool camOk = false;
static char camMsg[48] = "not started";

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
// Brightness of one pixel (the camera's bytes arrive swapped when read as uint16_t)
static inline uint8_t grayOf(uint16_t p) {
  uint16_t v = (uint16_t)((p << 8) | (p >> 8));
  uint8_t r = ((v >> 11) & 31) << 3;
  uint8_t g = ((v >> 5)  & 63) << 2;
  uint8_t b = (v & 31) << 3;
  return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

// 320x240 -> 160x120 for the stream
static void downscale2x(const camera_fb_t *fb, uint16_t *dst) {
  const uint16_t *src = (const uint16_t *)fb->buf;
  size_t w = fb->width, h = fb->height;
  for (size_t y = 0; y + 1 < h && (y / 2) < OUT_H; y += 2) {
    const uint16_t *row = src + y * w;
    for (size_t x = 0; x + 1 < w && (x / 2) < OUT_W; x += 2) *dst++ = row[x];
  }
}

static inline void putPx(int x, int y, uint16_t col) {
  if (x >= 0 && x < OUT_W && y >= 0 && y < OUT_H) workBuf[y * OUT_W + x] = col;
}

static void drawBox(int x0, int y0, int x1, int y1, uint16_t col) {
  for (int x = x0; x <= x1; x++) { putPx(x, y0, col); putPx(x, y1, col); }
  for (int y = y0; y <= y1; y++) { putPx(x0, y, col); putPx(x1, y, col); }
}

static void initBackground(const camera_fb_t *fb) {
  const uint16_t *src = (const uint16_t *)fb->buf;
  for (int i = 0; i < DET_W * DET_H; i++) bg[i] = grayOf(src[i]);
}

// Find the largest connected group of changed cells. Results are in 160x120 coordinates.
static Det detect(const camera_fb_t *fb, bool &globalChange) {
  Det d = { false, 0, 0, 0, 0, 0 };
  globalChange = false;
  memset(cellCnt, 0, sizeof(cellCnt));
  const uint16_t *src = (const uint16_t *)fb->buf;
  uint32_t total = 0;

  for (int y = 0; y < DET_H; y++) {
    const uint16_t *row = src + y * DET_W;
    uint8_t *b  = bg + y * DET_W;
    uint8_t *cr = cellCnt[y / CELL];
    for (int x = 0; x < DET_W; x++) {
      int g  = grayOf(row[x]);
      int bv = b[x];
      int diff = g - bv;
      if (diff > DIFF_THRESH || diff < -DIFF_THRESH) {
        cr[x / CELL]++;
        total++;
      }
      b[x] = (uint8_t)(bv + (diff > 0) - (diff < 0));   // background drifts toward the scene by 1 per frame
    }
  }

  if (total > GLOBAL_FRAC * DET_W * DET_H) {            // whole image changed: lighting or auto-exposure
    globalChange = true;
    initBackground(fb);
    return d;
  }

  memset(cellMark, 0, sizeof(cellMark));
  uint32_t bestSum = 0;
  float bx = 0, by = 0, bx2 = 0, by2 = 0;

  for (int gy = 0; gy < GRID_H; gy++) {
    for (int gx = 0; gx < GRID_W; gx++) {
      if (cellMark[gy][gx] || cellCnt[gy][gx] < CELL_MIN_PIXELS) continue;

      int sp = 0;
      cellStack[sp++] = gy * GRID_W + gx;
      cellMark[gy][gx] = 1;
      uint32_t sum = 0;
      float sx = 0, sy = 0, sx2 = 0, sy2 = 0;

      while (sp > 0) {
        int id = cellStack[--sp];
        int cy = id / GRID_W, cx = id % GRID_W;
        float w  = cellCnt[cy][cx];
        float px = cx * CELL + CELL / 2.0f;
        float py = cy * CELL + CELL / 2.0f;
        sum += (uint32_t)w;
        sx += w * px;       sy += w * py;
        sx2 += w * px * px; sy2 += w * py * py;

        for (int dy = -1; dy <= 1; dy++) {
          for (int dx = -1; dx <= 1; dx++) {
            int ny = cy + dy, nx = cx + dx;
            if (ny < 0 || ny >= GRID_H || nx < 0 || nx >= GRID_W) continue;
            if (cellMark[ny][nx] || cellCnt[ny][nx] < CELL_MIN_PIXELS) continue;
            cellMark[ny][nx] = 1;
            cellStack[sp++] = ny * GRID_W + nx;
          }
        }
      }
      if (sum > bestSum) { bestSum = sum; bx = sx / sum; by = sy / sum; bx2 = sx2 / sum; by2 = sy2 / sum; }
    }
  }

  if (bestSum >= MIN_TOTAL_PIXELS) {
    float vx = bx2 - bx * bx, vy = by2 - by * by;
    d.found = true;
    d.n  = bestSum;
    d.cx = bx / 2.0f;                                   // convert 320x240 -> 160x120 coordinates
    d.cy = by / 2.0f;
    d.sx = sqrtf(vx > 0 ? vx : 0) / 2.0f;
    d.sy = sqrtf(vy > 0 ? vy : 0) / 2.0f;
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
    if (fb->width != DET_W || fb->height != DET_H) {
      esp_camera_fb_return(fb);
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    Det d = { false, 0, 0, 0, 0, 0 };
    if (warmup < 10) {                                  // the first frames just build the background
      initBackground(fb);
      warmup++;
    } else {
      bool globalChange;
      d = detect(fb, globalChange);
    }
    downscale2x(fb, workBuf);
    esp_camera_fb_return(fb);

    if (d.found) {
      lostFrames = 0;
      filtX = (filtX < 0) ? d.cx : 0.6f * filtX + 0.4f * d.cx;

      float target = 90.0f + SERVO_DIR * ((filtX - OUT_W / 2.0f) / OUT_W) * CAM_FOV_DEG;
      target = constrain(target, 0.0f, 180.0f);
      float diff = target - servoAngle;
      if (fabsf(diff) > DEADBAND_DEG) {
        servoAngle += constrain(diff, -MAX_STEP_DEG, MAX_STEP_DEG);
        servoWrite((int)servoAngle);
      }

      float hw = max(2.0f * d.sx, 4.0f), hh = max(2.0f * d.sy, 4.0f);
      drawBox(constrain((int)(d.cx - hw), 0, OUT_W - 1), constrain((int)(d.cy - hh), 0, OUT_H - 1),
              constrain((int)(d.cx + hw), 0, OUT_W - 1), constrain((int)(d.cy + hh), 0, OUT_H - 1), COL_GREEN);
      for (int k = -4; k <= 4; k++) {
        putPx((int)d.cx + k, (int)d.cy, COL_RED);
        putPx((int)d.cx, (int)d.cy + k, COL_RED);
      }

      if (millis() - lastPrint > 500) {
        lastPrint = millis();
        Serial.printf("Object at x=%.0f y=%.0f (%u px changed)  servo=%d deg\n",
                      d.cx, d.cy, (unsigned)d.n, (int)servoAngle);
      }
    } else if (++lostFrames > 30) {
      filtX = -1;
    }

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
    if (frameId == lastId) { delay(2); continue; }

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
  config.frame_size   = FRAMESIZE_QVGA;      // 320x240
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    snprintf(camMsg, sizeof(camMsg), "init failed 0x%x", err);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  snprintf(camMsg, sizeof(camMsg), "OK (sensor PID 0x%x)", s->id.PID);
  return true;
}

static void *allocBuf(size_t n, bool preferPsram) {
  void *p = nullptr;
  if (preferPsram) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  if (!p) p = heap_caps_malloc(n, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (!p) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  return p;
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);                        // never stall when no monitor is connected
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 8000) delay(10);   // wait up to 8 s for the monitor to reconnect
  delay(300);

  Serial.printf("PSRAM: %s\n", psramFound() ? "found" : "NOT found");

  workBuf  = (uint16_t *)allocBuf(NPIX * 2, false);
  shareBuf = (uint16_t *)allocBuf(NPIX * 2, true);
  sendBuf  = (uint16_t *)allocBuf(NPIX * 2, true);
  bg       = (uint8_t  *)allocBuf(DET_W * DET_H, false);
  if (!workBuf || !shareBuf || !sendBuf || !bg) {
    Serial.println("Buffer allocation failed.");
    snprintf(camMsg, sizeof(camMsg), "buffer allocation failed");
    return;
  }
  memset(workBuf, 0, NPIX * 2);
  memset(shareBuf, 0, NPIX * 2);
  memset(bg, 0, DET_W * DET_H);
  frameMutex = xSemaphoreCreateMutex();

  camOk = startCamera();
  Serial.printf("Camera: %s\n", camMsg);
  if (!camOk) {
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

  // Repeat the status every 3 s so it shows up even if the monitor connected late
  // static uint32_t lastStatus = 0;
  // if (millis() - lastStatus > 3000) {
  //   lastStatus = millis();
  //   Serial.printf("[status] camera: %s | frames: %u | free heap: %u | open http://%s\n",
  //                 camMsg, (unsigned)frameId, (unsigned)esp_get_free_heap_size(),
  //                 WiFi.softAPIP().toString().c_str());
  // }
}