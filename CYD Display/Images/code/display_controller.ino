#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <TAMC_GT911.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// Demo-Modus: 1 = Chart ohne Heizplatte testen, 0 = normaler Betrieb
#define DEMO_MODE 0

// ── BLE NUS UUIDs ──────────────────────────────────────────────────────────
static BLEUUID kServiceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID kRxUuid     ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID kTxUuid     ("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
static const char* kTargetName = "ESP32";

// ── Farb-Palette (Midnight Amber) ──────────────────────────────────────────
#define COL_BG        0x080E1A  // Fast-Schwarz blau
#define COL_CARD      0x111D30  // Tiefes Navy
#define COL_BORDER    0x1E3655  // Gedämpftes Stahlblau
#define COL_TITLE     0xC8D8F0  // Eisblau-Weiss
#define COL_VAL       0xF5B731  // Amber/Gold
#define COL_VAL2      0xE0E8FF  // Hellgrau-weiss
#define COL_BTN       0x16273F  // Dunkles Blau
#define COL_BTN_ACT   0x1C4A7A  // Aktiv-Blau
#define COL_BTN_RED   0x6B1220  // Tiefrot
#define COL_BTN_RED_A 0x9B1C2F
#define COL_BTN_GREEN 0x0D3D22  // Tiefgrün
#define COL_BTN_GR_A  0x145C34
#define COL_ACCENT    0xF5B731  // Gold-Akzent
#define COL_KB_BG     0x0A1520  // Numpad-Hintergrund
#define COL_KB_BTN    0x152338  // Numpad-Taste
#define COL_KB_BTN_A  0x213D60  // Numpad gedrückt
#define COL_KB_DEL    0x3D1520
#define COL_KB_OK     0x0D3D22

TFT_eSPI tft = TFT_eSPI();

// ── GT911 Touch ────────────────────────────────────────────────────────────
#define TOUCH_GT_SDA  33
#define TOUCH_GT_SCL  32
#define TOUCH_GT_INT  21
#define TOUCH_GT_RST  25
TAMC_GT911 ts = TAMC_GT911(TOUCH_GT_SDA, TOUCH_GT_SCL, TOUCH_GT_INT, TOUCH_GT_RST, 320, 480);

BLEClient*               g_client       = nullptr;
BLERemoteCharacteristic* g_rxChar       = nullptr;
BLERemoteCharacteristic* g_txChar       = nullptr;
BLEAdvertisedDevice*     g_foundDevice  = nullptr;

bool   g_connected       = false;
bool   g_connectRequested = false;
float  g_realTemp        = NAN;
int    g_zend            = 120;
int    g_tend            = 180;
bool   g_fanState        = false;
bool   g_lightState      = false;
String g_status          = "Bereit";

unsigned long g_lastSyncMs   = 0;
uint32_t      g_lastLvTickMs = 0;

// ── LVGL Draw-Buffer ────────────────────────────────────────────────────────
static lv_color_t    g_buf1[320 * 10];
static lv_display_t* g_display     = nullptr;
static lv_indev_t*   g_inputDevice = nullptr;

// ── UI-Objekte ───────────────────────────────────────────────────────────────
static lv_obj_t* g_lblTemp   = nullptr;
static lv_obj_t* g_lblStatus = nullptr;
static lv_obj_t* g_lblBle    = nullptr;
static lv_obj_t* g_lblMeta   = nullptr;
static lv_obj_t* g_lblZend   = nullptr;
static lv_obj_t* g_lblTend   = nullptr;
static lv_obj_t* g_hdr       = nullptr;
static lv_obj_t* g_tempCard  = nullptr;
static lv_obj_t* g_actCard   = nullptr;
static lv_obj_t* g_numpad    = nullptr;  // Numpad-Overlay
static lv_obj_t* g_numInput  = nullptr;  // Anzeige-Label im Numpad
static bool      g_numpadForZend = true; // Welcher Wert wird bearbeitet?
static char      g_numBuf[8] = "";       // Eingabepuffer

static lv_obj_t* g_startOverlay = nullptr;
static lv_obj_t* g_startCurveGlow = nullptr;
static lv_obj_t* g_startCurve = nullptr;
static lv_obj_t* g_startTitle   = nullptr;
static bool      g_startAnimActive = false;
static uint32_t  g_startAnimMs  = 0;

// ── Trend-Chart ─────────────────────────────────────────────────────────────
#define CHART_MAX_PTS  120           // max. gespeicherte Datenpunkte
static lv_obj_t*   g_paramCard   = nullptr;
static lv_obj_t*   g_chartCard   = nullptr;
static lv_obj_t*   g_chart       = nullptr;
static lv_chart_series_t* g_chartSer = nullptr;
static lv_obj_t*   g_btnChartFs = nullptr;
static lv_obj_t*   g_lblChartFsBtn = nullptr;
static lv_obj_t*   g_lblChartXMin = nullptr;
static lv_obj_t*   g_lblChartXMid = nullptr;
static lv_obj_t*   g_lblChartXMax = nullptr;
static lv_obj_t*   g_lblChartYMax = nullptr;
static lv_obj_t*   g_lblChartYMid = nullptr;
static lv_obj_t*   g_lblChartYMin = nullptr;
static lv_obj_t*   g_chartMarkerLine = nullptr;
static lv_obj_t*   g_chartLastDot = nullptr;
static lv_obj_t*   g_chartLastLabel = nullptr;
static lv_obj_t*   g_chartFsOverlay = nullptr;
static lv_obj_t*   g_chartFs = nullptr;
static lv_chart_series_t* g_chartFsSer = nullptr;
static lv_obj_t*   g_btnChartFsClose = nullptr;
static lv_obj_t*   g_lblChartFsClose = nullptr;
static lv_obj_t*   g_lblChartFsXMin = nullptr;
static lv_obj_t*   g_lblChartFsXMid = nullptr;
static lv_obj_t*   g_lblChartFsXMax = nullptr;
static lv_obj_t*   g_lblChartFsYMax = nullptr;
static lv_obj_t*   g_lblChartFsYMid = nullptr;
static lv_obj_t*   g_lblChartFsYMin = nullptr;
static lv_obj_t*   g_chartFsMarkerLine = nullptr;
static lv_obj_t*   g_chartFsLastDot = nullptr;
static lv_obj_t*   g_chartFsLastLabel = nullptr;
static bool        g_chartFullscreen = false;
static lv_obj_t*   g_cbDemo      = nullptr;
static bool        g_heatingActive = false;
static uint32_t    g_heatStartMs  = 0;
static uint32_t    g_chartSpanSec = 120;
static uint16_t    g_chartPointTarget = CHART_MAX_PTS;
static float       g_chartTempBuf[CHART_MAX_PTS];
static uint16_t    g_chartPtCount = 0;
static int         g_chartYMax = 200;
static uint32_t    g_lastChartMs  = 0;

static bool        g_demoEnabled = (DEMO_MODE == 1);
static float       g_demoAmbient = 25.0f;

// Board-RGB-LED (kein Neopixel): alternativ gemeldete Pins 17, 4, 16
// Reihenfolge ist vorerst angenommen und bei Bedarf schnell tauschbar.
#define STATUS_LED_R_PIN 4
#define STATUS_LED_G_PIN 16
#define STATUS_LED_B_PIN 17
// Viele integrierte RGB-LEDs sind low-active (0 = an, 1 = aus).
#define STATUS_LED_ACTIVE_LOW 1
// GPIO1 wird oft als UART-TX verwendet; fuer LED-Nutzung die serielle Ausgabe beenden.
#define STATUS_LED_RELEASE_GPIO1_FROM_SERIAL 0
// Globale Dimmung fuer die Status-LED (0..255). Klein halten, damit die LED nicht blendet.
#define STATUS_LED_DIM 40

// Schwache RGB-LED-Zustaende (falls RGB_BUILTIN vorhanden)
static void applyStatusLed();

// ── Styles ──────────────────────────────────────────────────────────────────
static lv_style_t sScreen, sCard, sTitle, sBig, sVal,
                  sBtn, sBtnDanger, sBtnOk, sNumBtn,
                  sNumBtnDel, sNumBtnOk, sNumpad;

static float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

static float easeOutCubic(float value) {
  value = clamp01(value);
  float inv = 1.0f - value;
  return 1.0f - (inv * inv * inv);
}

static const uint16_t kStartCurvePointCount = 48;
static lv_point_precise_t g_startCurveBase[kStartCurvePointCount];
static lv_point_precise_t g_startCurvePts[kStartCurvePointCount];

static float sampleStartReflowProfile(float t) {
  t = clamp01(t);

  if (t < 0.38f) {
    float ramp = t / 0.38f;
    return 0.82f - 0.34f * easeOutCubic(ramp);
  }

  if (t < 0.62f) {
    float soak = (t - 0.38f) / 0.24f;
    return 0.48f + 0.02f * sinf(soak * 3.14159f);
  }

  if (t < 0.82f) {
    float peak = (t - 0.62f) / 0.20f;
    return 0.48f - 0.34f * easeOutCubic(peak);
  }

  float cool = (t - 0.82f) / 0.18f;
  return 0.14f + 0.56f * clamp01(cool);
}

static void generateStartCurveBase() {
  for (uint16_t i = 0; i < kStartCurvePointCount; ++i) {
    float t = (float)i / (float)(kStartCurvePointCount - 1U);
    g_startCurveBase[i].x = (lv_value_precise_t)(236.0f * t + 0.5f);
    g_startCurveBase[i].y = (lv_value_precise_t)(sampleStartReflowProfile(t) * 82.0f + 0.5f);
  }
}

static void updateStartCurvePoints(float progress) {
  if (kStartCurvePointCount == 0) return;

  progress = clamp01(progress);
  float segmentPos = progress * (float)(kStartCurvePointCount - 1U);
  uint32_t lastFull = (uint32_t)segmentPos;
  float frac = segmentPos - (float)lastFull;

  g_startCurvePts[0] = g_startCurveBase[0];
  for (uint32_t i = 1; i < kStartCurvePointCount; ++i) {
    if (i < lastFull) {
      g_startCurvePts[i] = g_startCurveBase[i];
      continue;
    }

    if (i == lastFull && i < kStartCurvePointCount - 1U) {
      lv_point_precise_t from = g_startCurveBase[i];
      lv_point_precise_t to   = g_startCurveBase[i + 1U];
      g_startCurvePts[i].x = from.x + (lv_value_precise_t)((to.x - from.x) * frac);
      g_startCurvePts[i].y = from.y + (lv_value_precise_t)((to.y - from.y) * frac);
      continue;
    }

    g_startCurvePts[i] = g_startCurvePts[i - 1];
  }
}

static void setRevealState(lv_obj_t* obj, float progress, lv_coord_t startOffset) {
  if (!obj) return;
  float eased = easeOutCubic(progress);
  lv_opa_t opa = (lv_opa_t)(LV_OPA_COVER * eased);
  lv_coord_t translateY = (lv_coord_t)((1.0f - eased) * (float)startOffset);
  lv_obj_set_style_opa(obj, opa, 0);
  lv_obj_set_style_translate_y(obj, translateY, 0);
}

static void beginStartupAnimation(lv_obj_t* parent) {
  if (!parent) return;

  g_startOverlay = lv_obj_create(parent);
  lv_obj_set_size(g_startOverlay, lv_pct(100), lv_pct(100));
  lv_obj_align(g_startOverlay, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(g_startOverlay, lv_color_hex(COL_BG), 0);
  lv_obj_set_style_bg_opa(g_startOverlay, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(g_startOverlay, 0, 0);
  lv_obj_set_style_radius(g_startOverlay, 0, 0);
  lv_obj_clear_flag(g_startOverlay, LV_OBJ_FLAG_SCROLLABLE);

  generateStartCurveBase();
  updateStartCurvePoints(0.0f);

  g_startCurveGlow = lv_line_create(g_startOverlay);
  lv_line_set_points(g_startCurveGlow, g_startCurvePts,
                     kStartCurvePointCount);
  lv_obj_set_size(g_startCurveGlow, 236, 82);
  lv_obj_align(g_startCurveGlow, LV_ALIGN_CENTER, 0, -28);
  lv_obj_set_style_line_color(g_startCurveGlow, lv_color_hex(0x8A5D00), 0);
  lv_obj_set_style_line_opa(g_startCurveGlow, LV_OPA_30, 0);
  lv_obj_set_style_line_width(g_startCurveGlow, 5, 0);
  lv_obj_set_style_line_rounded(g_startCurveGlow, true, 0);
  lv_obj_set_style_bg_opa(g_startCurveGlow, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(g_startCurveGlow, 0, 0);

  g_startCurve = lv_line_create(g_startOverlay);
  lv_line_set_points(g_startCurve, g_startCurvePts,
                     kStartCurvePointCount);
  lv_obj_set_size(g_startCurve, 236, 82);
  lv_obj_align(g_startCurve, LV_ALIGN_CENTER, 0, -28);
  lv_obj_set_style_line_color(g_startCurve, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_line_opa(g_startCurve, LV_OPA_COVER, 0);
  lv_obj_set_style_line_width(g_startCurve, 2, 0);
  lv_obj_set_style_line_rounded(g_startCurve, true, 0);
  lv_obj_set_style_bg_opa(g_startCurve, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(g_startCurve, 0, 0);

  g_startTitle = lv_label_create(g_startOverlay);
  lv_label_set_text(g_startTitle, "BLEIron");
  lv_obj_add_style(g_startTitle, &sBig, 0);
  lv_obj_set_style_text_color(g_startTitle, lv_color_hex(COL_TITLE), 0);
  lv_obj_set_style_text_opa(g_startTitle, LV_OPA_TRANSP, 0);
  lv_obj_set_style_translate_y(g_startTitle, 10, 0);
  lv_obj_align(g_startTitle, LV_ALIGN_CENTER, 0, 58);

  setRevealState(g_hdr, 0.0f, 10);
  setRevealState(g_tempCard, 0.0f, 14);
  setRevealState(g_paramCard, 0.0f, 18);
  setRevealState(g_actCard, 0.0f, 22);

  g_startAnimActive = true;
  g_startAnimMs = millis();
}

static void updateStartupAnimation() {
  if (!g_startAnimActive) return;

  uint32_t elapsed = millis() - g_startAnimMs;

  // Phase 1 (0..1350ms): Reflow-Kurve zeichnet sich auf
  if (g_startCurve || g_startCurveGlow) {
    float curveP = easeOutCubic((elapsed - 80.0f) / 1150.0f);
    updateStartCurvePoints(curveP);
    if (g_startCurveGlow) lv_obj_invalidate(g_startCurveGlow);
    if (g_startCurve) lv_obj_invalidate(g_startCurve);
  }

  // Phase 1b (500..1300ms): Titel einblenden
  if (g_startTitle) {
    float titleInP  = easeOutCubic((elapsed -  500.0f) / 500.0f);
    // Phase 2 (1350..1650ms): Titel wieder ausblenden
    float titleOutP = clamp01((elapsed - 1350.0f) / 280.0f);
    float titleOpa  = titleInP * (1.0f - titleOutP);
    lv_obj_set_style_text_opa(g_startTitle, (lv_opa_t)(LV_OPA_COVER * clamp01(titleOpa)), 0);
    lv_obj_set_style_translate_y(g_startTitle, (lv_coord_t)((1.0f - titleInP) * 10.0f), 0);
  }

  // Phase 2 (1300..1700ms): Kurve und Overlay-Hintergrund ausblenden
  if (g_startCurve) {
    float fadeP = clamp01((elapsed - 1300.0f) / 360.0f);
    lv_obj_set_style_line_opa(g_startCurve, (lv_opa_t)(LV_OPA_COVER * (1.0f - fadeP)), 0);
  }
  if (g_startCurveGlow) {
    float fadeP = clamp01((elapsed - 1300.0f) / 360.0f);
    lv_obj_set_style_line_opa(g_startCurveGlow, (lv_opa_t)(LV_OPA_30 * (1.0f - fadeP)), 0);
  }
  if (g_startOverlay) {
    float overlayP = clamp01((elapsed - 1650.0f) / 300.0f);
    lv_obj_set_style_bg_opa(g_startOverlay,
                            (lv_opa_t)(LV_OPA_COVER * (1.0f - overlayP)), 0);
  }

  // Phase 3 (1950..2450ms): UI einblenden — erst wenn Overlay weg ist
  setRevealState(g_hdr,      (elapsed - 1950.0f) / 220.0f, 10);
  setRevealState(g_tempCard, (elapsed - 2060.0f) / 240.0f, 14);
  setRevealState(g_paramCard,(elapsed - 2170.0f) / 240.0f, 18);
  setRevealState(g_actCard,  (elapsed - 2280.0f) / 240.0f, 22);

  if (elapsed >= 2600U) {
    setRevealState(g_hdr, 1.0f, 0);
    setRevealState(g_tempCard, 1.0f, 0);
    setRevealState(g_paramCard, 1.0f, 0);
    setRevealState(g_actCard, 1.0f, 0);
    if (g_startOverlay) {
      lv_obj_delete(g_startOverlay);
      g_startOverlay = nullptr;
      g_startCurveGlow = nullptr;
      g_startCurve = nullptr;
      g_startTitle = nullptr;
    }
    g_startAnimActive = false;
  }
}

// ============================================================================
// BLE-Callbacks
// ============================================================================
class ClientCbs : public BLEClientCallbacks {
 public:
  void onConnect(BLEClient*)    override {
    g_connected = true;
    g_status = "Verbunden";
    applyStatusLed();
  }
  void onDisconnect(BLEClient*) override {
    g_connected = false; g_rxChar = nullptr; g_txChar = nullptr;
    g_status = "Getrennt";
    applyStatusLed();
  }
};

class ScanCbs : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    bool nm = d.haveName() && d.getName() == kTargetName;
    bool sm = d.haveServiceUUID() && d.isAdvertisingService(kServiceUuid);
    if (nm || sm) { g_foundDevice = new BLEAdvertisedDevice(d); BLEDevice::getScan()->stop(); }
  }
};

static void notifyCallback(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  String p; p.reserve(len + 1);
  for (size_t i = 0; i < len; i++) p += (char)data[i];
  p.trim();
  if (p.length()) g_realTemp = p.toFloat();
}

// ============================================================================
// LVGL-Callbacks
// ============================================================================
void lvFlushCb(lv_display_t* disp, const lv_area_t* area, uint8_t* px) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t*)px, w * h, true);
  tft.endWrite();
  lv_display_flush_ready(disp);
}

void lvTouchReadCb(lv_indev_t* indev, lv_indev_data_t* data) {
  (void)indev;
  ts.read();
  if (ts.isTouched) {
    lv_coord_t rawX = (lv_coord_t)ts.points[0].x;
    lv_coord_t rawY = (lv_coord_t)ts.points[0].y;

    data->state   = LV_INDEV_STATE_PRESSED;

    if (g_chartFullscreen) {
      // Portrait-Touchkoordinaten (320x480) in Landscape (480x320) spiegeln/abbilden.
      lv_coord_t w = (lv_coord_t)tft.width();
      lv_coord_t h = (lv_coord_t)tft.height();
      lv_coord_t mappedX = rawY;
      lv_coord_t mappedY = (lv_coord_t)(h - 1 - rawX);

      if (mappedX < 0) mappedX = 0;
      if (mappedX >= w) mappedX = (lv_coord_t)(w - 1);
      if (mappedY < 0) mappedY = 0;
      if (mappedY >= h) mappedY = (lv_coord_t)(h - 1);

      data->point.x = mappedX;
      data->point.y = mappedY;
    } else {
      data->point.x = rawX;
      data->point.y = rawY;
    }
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// ============================================================================
// BLE-Kommando senden
// ============================================================================
bool sendCommand(const String& cmd) {
  if (!g_connected || !g_rxChar || !g_rxChar->canWrite()) {
    g_status = "Nicht verbunden";
    return false;
  }
  g_rxChar->writeValue((uint8_t*)cmd.c_str(), cmd.length(), true);
  return true;
}

// ============================================================================
// UI-Text synchronisieren
// ============================================================================
void syncUiText() {
  if (g_lblTemp) {
    String t = isnan(g_realTemp) ? "--.-" : String(g_realTemp, 1);
    lv_label_set_text(g_lblTemp, (t + " C").c_str());
  }
  if (g_lblStatus) lv_label_set_text(g_lblStatus, g_status.c_str());
  if (g_lblBle) {
    lv_label_set_text(g_lblBle, g_connected ? "BLE: verbunden" : "BLE: getrennt");
  }
  if (g_lblMeta) {
    String m = String("Licht ") + (g_lightState ? "AN" : "AUS")
             + "  |  Luefter " + (g_fanState ? "AN" : "AUS");
    lv_label_set_text(g_lblMeta, m.c_str());
  }
  if (g_lblZend) lv_label_set_text(g_lblZend, String(g_zend).c_str());
  if (g_lblTend) lv_label_set_text(g_lblTend, String(g_tend).c_str());
}

static void setChartButtonLabel(lv_obj_t* lbl, const char* text) {
  if (lbl) lv_label_set_text(lbl, text);
}

static void layoutFullscreenChartUi() {
  if (!g_chartFsOverlay || !g_chartFs) return;

  lv_obj_t* scr = lv_screen_active();
  if (!scr) return;

  lv_coord_t w = lv_obj_get_width(scr);
  lv_coord_t h = lv_obj_get_height(scr);

  lv_obj_set_size(g_chartFsOverlay, w, h);

  lv_coord_t leftAxisSpace = 26;
  lv_coord_t rightPad = 34;
  lv_coord_t chartTop = 42;
  lv_coord_t chartBottom = 26;
  lv_coord_t chartX = leftAxisSpace;
  lv_coord_t chartW = w - leftAxisSpace - rightPad;
  lv_coord_t chartH = h - chartTop - chartBottom;
  if (chartW < 140) chartW = 140;
  if (chartH < 100) chartH = 100;

  lv_obj_set_size(g_chartFs, chartW, chartH);
  lv_obj_set_pos(g_chartFs, chartX, chartTop);

  if (g_lblChartFsXMin) lv_obj_align_to(g_lblChartFsXMin, g_chartFs, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 4);
  if (g_lblChartFsXMid) lv_obj_align_to(g_lblChartFsXMid, g_chartFs, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
  if (g_lblChartFsXMax) lv_obj_align_to(g_lblChartFsXMax, g_chartFs, LV_ALIGN_OUT_BOTTOM_RIGHT, 0, 4);

  if (g_lblChartFsYMax) lv_obj_align_to(g_lblChartFsYMax, g_chartFs, LV_ALIGN_OUT_LEFT_TOP, -2, 0);
  if (g_lblChartFsYMid) lv_obj_align_to(g_lblChartFsYMid, g_chartFs, LV_ALIGN_OUT_LEFT_MID, -2, 0);
  if (g_lblChartFsYMin) lv_obj_align_to(g_lblChartFsYMin, g_chartFs, LV_ALIGN_OUT_LEFT_BOTTOM, -2, 0);
}

static void setFullscreenDisplayRotation(bool enable) {
  if (!g_display) return;

  if (enable) {
    tft.setRotation(1);
    ts.setRotation(ROTATION_INVERTED);
  } else {
    tft.setRotation(0);
    ts.setRotation(ROTATION_INVERTED);
  }

  lv_display_set_resolution(g_display, tft.width(), tft.height());
}

static void setChartFullscreen(bool enable) {
  g_chartFullscreen = enable;
  if (g_chartFsOverlay) {
    if (enable) {
      setFullscreenDisplayRotation(true);
      layoutFullscreenChartUi();
      lv_obj_clear_flag(g_chartFsOverlay, LV_OBJ_FLAG_HIDDEN);
      lv_obj_move_foreground(g_chartFsOverlay);
      refreshChartsFromBuffer();
    } else {
      lv_obj_add_flag(g_chartFsOverlay, LV_OBJ_FLAG_HIDDEN);
      setFullscreenDisplayRotation(false);
    }
  }
  setChartButtonLabel(g_lblChartFsBtn, enable ? "MIN" : "MAX");
  setChartButtonLabel(g_lblChartFsClose, "MIN");
}

static void onChartFullscreenToggle(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  setChartFullscreen(!g_chartFullscreen);
}

static void clearChartBuffer() {
  for (uint16_t i = 0; i < CHART_MAX_PTS; ++i) g_chartTempBuf[i] = NAN;
  g_chartPtCount = 0;
}

static void refreshChartFromBuffer(lv_obj_t* chart, lv_chart_series_t* ser) {
  if (!chart || !ser) return;
  lv_chart_set_all_values(chart, ser, LV_CHART_POINT_NONE);
  for (uint16_t i = 0; i < g_chartPtCount && i < g_chartPointTarget; ++i) {
    if (!isnan(g_chartTempBuf[i])) {
      lv_chart_set_value_by_id(chart, ser, i, (lv_coord_t)g_chartTempBuf[i]);
    }
  }
  lv_chart_refresh(chart);
}

static void refreshChartsFromBuffer() {
  refreshChartFromBuffer(g_chart, g_chartSer);
  refreshChartFromBuffer(g_chartFs, g_chartFsSer);
  updateMarkerLine(g_chart, &g_chartMarkerLine, g_chartYMax, g_tend);
  updateMarkerLine(g_chartFs, &g_chartFsMarkerLine, g_chartYMax, g_tend);
  updateLastTempOverlay(g_chartFs, &g_chartFsLastDot, &g_chartFsLastLabel, g_chartYMax);

  if (g_chartLastDot) lv_obj_add_flag(g_chartLastDot, LV_OBJ_FLAG_HIDDEN);
  if (g_chartLastLabel) lv_obj_add_flag(g_chartLastLabel, LV_OBJ_FLAG_HIDDEN);
}

static void updateChartAxisLabels(uint32_t spanSec, int yMax, int yMid) {
  char buf[24];

  if (g_lblChartXMin) lv_label_set_text(g_lblChartXMin, "");
  if (g_lblChartFsXMin) lv_label_set_text(g_lblChartFsXMin, "");

  snprintf(buf, sizeof(buf), "%lus", (unsigned long)(spanSec / 2U));
  if (g_lblChartXMid) lv_label_set_text(g_lblChartXMid, buf);
  if (g_lblChartFsXMid) lv_label_set_text(g_lblChartFsXMid, buf);

  snprintf(buf, sizeof(buf), "%lus", (unsigned long)spanSec);
  if (g_lblChartXMax) lv_label_set_text(g_lblChartXMax, buf);
  if (g_lblChartFsXMax) lv_label_set_text(g_lblChartFsXMax, buf);

  snprintf(buf, sizeof(buf), "%d C", yMax);
  if (g_lblChartYMax) lv_label_set_text(g_lblChartYMax, buf);
  if (g_lblChartFsYMax) lv_label_set_text(g_lblChartFsYMax, buf);

  snprintf(buf, sizeof(buf), "%d C", yMid);
  if (g_lblChartYMid) lv_label_set_text(g_lblChartYMid, buf);
  if (g_lblChartFsYMid) lv_label_set_text(g_lblChartFsYMid, buf);

  if (g_lblChartYMin) lv_label_set_text(g_lblChartYMin, "0 C");
  if (g_lblChartFsYMin) lv_label_set_text(g_lblChartFsYMin, "0 C");
}

static void updateMarkerLine(lv_obj_t* chart, lv_obj_t** markerLine, int yMax, int targetTemp) {
  if (!chart || !markerLine || yMax <= 0) return;

  lv_coord_t pad = 4;
  lv_coord_t chartW = lv_obj_get_width(chart);
  lv_coord_t chartH = lv_obj_get_height(chart);
  lv_coord_t innerW = chartW - (pad * 2);
  lv_coord_t innerH = chartH - (pad * 2);
  if (innerW < 10 || innerH < 10) return;

  lv_coord_t yMarker = pad + (lv_coord_t)((float)(yMax - targetTemp) / (float)yMax * (float)innerH);
  if (yMarker < pad) yMarker = pad;
  if (yMarker > (pad + innerH)) yMarker = pad + innerH;

  if (*markerLine == nullptr) {
    *markerLine = lv_obj_create(chart);
    lv_obj_set_style_bg_color(*markerLine, lv_color_hex(0xFF3333), 0);
    lv_obj_set_style_bg_opa(*markerLine, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(*markerLine, 0, 0);
    lv_obj_set_style_pad_all(*markerLine, 0, 0);
    lv_obj_clear_flag(*markerLine, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(*markerLine, LV_OBJ_FLAG_CLICKABLE);
  } else {
    lv_obj_clear_flag(*markerLine, LV_OBJ_FLAG_HIDDEN);
  }

  lv_obj_set_size(*markerLine, innerW, 1);
  lv_obj_set_pos(*markerLine, pad, yMarker);
}

static void updateLastTempOverlay(lv_obj_t* chart,
                                  lv_obj_t** dot,
                                  lv_obj_t** label,
                                  int yMax) {
  if (!chart || !dot || !label || yMax <= 0 || g_chartPtCount == 0) return;

  lv_coord_t pad = 4;
  lv_coord_t chartW = lv_obj_get_width(chart);
  lv_coord_t chartH = lv_obj_get_height(chart);
  lv_coord_t innerW = chartW - (pad * 2);
  lv_coord_t innerH = chartH - (pad * 2);
  if (innerW < 12 || innerH < 12) return;

  uint16_t lastIdx = (g_chartPtCount > 0) ? (g_chartPtCount - 1U) : 0U;
  if (lastIdx >= g_chartPointTarget) lastIdx = g_chartPointTarget - 1U;
  float lastTemp = g_chartTempBuf[lastIdx];
  if (isnan(lastTemp)) return;

  float xNorm = (g_chartPointTarget > 1U)
    ? ((float)lastIdx / (float)(g_chartPointTarget - 1U))
    : 0.0f;
  lv_coord_t x = pad + (lv_coord_t)(xNorm * (float)innerW);
  lv_coord_t y = pad + (lv_coord_t)(((float)yMax - lastTemp) / (float)yMax * (float)innerH);
  if (x < pad) x = pad;
  if (x > (pad + innerW)) x = pad + innerW;
  if (y < pad) y = pad;
  if (y > (pad + innerH)) y = pad + innerH;

  if (*dot == nullptr) {
    *dot = lv_obj_create(chart);
    lv_obj_set_size(*dot, 6, 6);
    lv_obj_set_style_radius(*dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(*dot, lv_color_hex(COL_ACCENT), 0);
    lv_obj_set_style_bg_opa(*dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(*dot, 0, 0);
    lv_obj_clear_flag(*dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(*dot, LV_OBJ_FLAG_CLICKABLE);
  }

  if (*label == nullptr) {
    *label = lv_label_create(chart);
    lv_obj_add_style(*label, &sTitle, 0);
    lv_obj_set_style_text_color(*label, lv_color_hex(COL_ACCENT), 0);
    lv_obj_clear_flag(*label, LV_OBJ_FLAG_CLICKABLE);
  }

  lv_obj_set_pos(*dot, x - 3, y - 3);

  char buf[20];
  snprintf(buf, sizeof(buf), "%.1f C", lastTemp);
  lv_label_set_text(*label, buf);
  lv_obj_update_layout(*label);

  lv_coord_t lblW = lv_obj_get_width(*label);
  lv_coord_t lblH = lv_obj_get_height(*label);
  lv_coord_t lblX = x + 8;
  lv_coord_t lblY = y - lblH - 6;
  if (lblX + lblW > chartW - 2) lblX = x - lblW - 8;
  if (lblX < 2) lblX = 2;
  if (lblY < 2) lblY = y + 8;
  if (lblY + lblH > chartH - 2) lblY = chartH - lblH - 2;
  lv_obj_set_pos(*label, lblX, lblY);
}

static void prepareChartsForRun() {
  g_chartSpanSec  = (uint32_t)g_zend * 2U;
  if (g_chartSpanSec < 1U) g_chartSpanSec = 1U;

  uint16_t pts = (uint16_t)((g_chartSpanSec / 2U) + 1U);
  if (pts < 2U) pts = 2U;
  if (pts > CHART_MAX_PTS) pts = CHART_MAX_PTS;
  g_chartPointTarget = pts;

  g_chartYMax = (int)(g_tend * 1.1f + 10.0f);
  if (g_chartYMax < 20) g_chartYMax = 20;
  int yMid = g_chartYMax / 2;

  if (g_chart) {
    lv_chart_set_range(g_chart, LV_CHART_AXIS_PRIMARY_Y, 0, (lv_coord_t)g_chartYMax);
    lv_chart_set_point_count(g_chart, g_chartPointTarget);
  }
  if (g_chartFs) {
    lv_chart_set_range(g_chartFs, LV_CHART_AXIS_PRIMARY_Y, 0, (lv_coord_t)g_chartYMax);
    lv_chart_set_point_count(g_chartFs, g_chartPointTarget);
  }

  updateChartAxisLabels(g_chartSpanSec, g_chartYMax, yMid);
  updateMarkerLine(g_chart, &g_chartMarkerLine, g_chartYMax, g_tend);
  updateMarkerLine(g_chartFs, &g_chartFsMarkerLine, g_chartYMax, g_tend);

  clearChartBuffer();
  if (isnan(g_realTemp)) g_realTemp = g_demoAmbient;
  g_chartTempBuf[0] = g_realTemp;
  g_chartPtCount = 1;
  refreshChartsFromBuffer();
  updateLastTempOverlay(g_chartFs, &g_chartFsLastDot, &g_chartFsLastLabel, g_chartYMax);
}

// ============================================================================
// Numpad
// ============================================================================
static void closeNumpad() {
  if (g_numpad) {
    lv_obj_del(g_numpad);
    g_numpad  = nullptr;
    g_numInput = nullptr;
  }
}

static void numpadBtnCb(lv_event_t* e) {
  const char* lbl = (const char*)lv_event_get_user_data(e);
  if (!lbl) return;

  if (strcmp(lbl, "OK") == 0) {
    int v = atoi(g_numBuf);
    if (v > 0) {
      if (g_numpadForZend) {
        g_zend = constrain(v, 60, 7200);
      } else {
        g_tend = constrain(v, 40, 350);
      }
    }
    closeNumpad();
    syncUiText();
    return;
  }
  if (strcmp(lbl, "X") == 0) {
    closeNumpad();
    return;
  }
  if (strcmp(lbl, "<") == 0) {
    int len = strlen(g_numBuf);
    if (len > 0) g_numBuf[len - 1] = '\0';
  } else {
    if (strlen(g_numBuf) < 5) {
      strncat(g_numBuf, lbl, 1);
    }
  }
  if (g_numInput) lv_label_set_text(g_numInput, g_numBuf[0] ? g_numBuf : "_");
}

static lv_obj_t* makeNumKey(lv_obj_t* parent, const char* txt,
                             lv_style_t* st, int col, int row,
                             int16_t kw, int16_t kh, int16_t gap) {
  lv_obj_t* btn = lv_button_create(parent);
  lv_obj_set_size(btn, kw, kh);
  lv_obj_set_pos(btn, col * (kw + gap), row * (kh + gap));
  lv_obj_add_style(btn, st, 0);
  lv_obj_add_event_cb(btn, numpadBtnCb, LV_EVENT_CLICKED,
                      (void*)txt);
  lv_obj_t* l = lv_label_create(btn);
  lv_label_set_text(l, txt);
  lv_obj_center(l);
  lv_obj_add_style(l, &sTitle, 0);
  return btn;
}

static void openNumpad(bool forZend) {
  if (g_numpad) closeNumpad();
  g_numpadForZend = forZend;
  memset(g_numBuf, 0, sizeof(g_numBuf));

  // Overlay-Container  (Padding=0, damit X-Button wirklich in die Ecke passt)
  g_numpad = lv_obj_create(lv_screen_active());
  lv_obj_set_size(g_numpad, 300, 330);
  lv_obj_align(g_numpad, LV_ALIGN_CENTER, 0, 10);
  lv_obj_add_style(g_numpad, &sNumpad, 0);
  lv_obj_set_style_pad_all(g_numpad, 0, 0);   // Padding entfernen – manuelles Layout
  lv_obj_clear_flag(g_numpad, LV_OBJ_FLAG_SCROLLABLE);

  // X-Schliessen-Button – echte obere rechte Ecke
  lv_obj_t* btnX = lv_button_create(g_numpad);
  lv_obj_set_size(btnX, 36, 30);
  lv_obj_align(btnX, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_set_style_bg_color(btnX, lv_color_hex(COL_BTN_RED), 0);
  lv_obj_set_style_radius(btnX, 10, 0);
  lv_obj_set_style_radius(btnX, 0, 0);   // oben rechts eckig (wie Container)
  lv_obj_add_event_cb(btnX, numpadBtnCb, LV_EVENT_CLICKED, (void*)"X");
  lv_obj_t* xl = lv_label_create(btnX);
  lv_label_set_text(xl, "X");
  lv_obj_add_style(xl, &sTitle, 0);
  lv_obj_center(xl);

  // Titel
  lv_obj_t* hdr = lv_label_create(g_numpad);
  lv_label_set_text(hdr, forZend ? "Dauer (60-7200 s)" : "Ziel-Temp (40-350 C)");
  lv_obj_add_style(hdr, &sTitle, 0);
  lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 6);

  // Eingabefeld
  lv_obj_t* inputBg = lv_obj_create(g_numpad);
  lv_obj_set_size(inputBg, 268, 44);
  lv_obj_align(inputBg, LV_ALIGN_TOP_MID, 0, 32);
  lv_obj_set_style_bg_color(inputBg, lv_color_hex(0x060C16), 0);
  lv_obj_set_style_border_color(inputBg, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_border_width(inputBg, 1, 0);
  lv_obj_set_style_radius(inputBg, 6, 0);
  lv_obj_set_style_pad_all(inputBg, 0, 0);
  lv_obj_clear_flag(inputBg, LV_OBJ_FLAG_SCROLLABLE);

  g_numInput = lv_label_create(inputBg);
  lv_label_set_text(g_numInput, "_");
  lv_obj_add_style(g_numInput, &sBig, 0);
  lv_obj_align(g_numInput, LV_ALIGN_CENTER, 0, 0);

  // Zahlen-Grid – exakt so breit wie 3 Tasten + 2 Lücken, zentriert
  // KW=84, KH=48, GAP=6 → Breite=84*3+6*2=264, Höhe=48*4+6*3=210
  const int16_t KW = 84, KH = 48, GAP = 6;
  const int16_t GRID_W = 3 * KW + 2 * GAP;  // 264
  const int16_t GRID_H = 4 * KH + 3 * GAP;  // 210

  lv_obj_t* grid = lv_obj_create(g_numpad);
  lv_obj_set_size(grid, GRID_W, GRID_H);
  lv_obj_align(grid, LV_ALIGN_TOP_MID, 0, 86);  // unter inputBg (32+44+10=86)
  lv_obj_set_style_bg_opa(grid, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(grid, 0, 0);
  lv_obj_set_style_pad_all(grid, 0, 0);
  lv_obj_clear_flag(grid, LV_OBJ_FLAG_SCROLLABLE);

  const char* keys[] = { "1","2","3", "4","5","6", "7","8","9", "<","0","OK" };
  lv_style_t* keyStyles[] = {
    &sNumBtn,&sNumBtn,&sNumBtn,
    &sNumBtn,&sNumBtn,&sNumBtn,
    &sNumBtn,&sNumBtn,&sNumBtn,
    &sNumBtnDel,&sNumBtn,&sNumBtnOk
  };
  for (int i = 0; i < 12; i++) {
    makeNumKey(grid, keys[i], keyStyles[i],
               i % 3, i / 3, KW, KH, GAP);
  }
}

// ============================================================================
// Haupt-Button-Events
// ============================================================================
void onBtnEvent(lv_event_t* e) {
  int id = (int)(uintptr_t)lv_event_get_user_data(e);
  switch (id) {
    case 0: openNumpad(true);   break;  // Zend-Feld angetippt
    case 1: openNumpad(false);  break;  // Tend-Feld angetippt
    case 2:
      {
        bool started = false;
        if (g_connected) {
          started = sendCommand("time," + String(g_zend) + ",temp," + String(g_tend));
        } else if (g_demoEnabled) {
          started = true;
        }

        if (started) {
          g_status = g_connected ? "Heizen gestartet" : "Demo gestartet";
        // Chart anzeigen, paramCard verstecken
        g_heatingActive = true;
        g_heatStartMs   = millis();
        g_lastChartMs   = millis();
        prepareChartsForRun();
        setChartFullscreen(false);
        if (g_paramCard) lv_obj_add_flag(g_paramCard, LV_OBJ_FLAG_HIDDEN);
        if (g_chartCard) lv_obj_clear_flag(g_chartCard, LV_OBJ_FLAG_HIDDEN);
        } else {
          g_status = "Nicht verbunden";
        }
      }
      break;
    case 3:
      if ((g_connected && sendCommand("A1")) || (!g_connected && g_demoEnabled)) {
        g_status = g_connected ? "Abbruch gesendet" : "Demo beendet";
        g_heatingActive = false;
        setChartFullscreen(false);
        if (g_chartCard) lv_obj_add_flag(g_chartCard, LV_OBJ_FLAG_HIDDEN);
        if (g_paramCard) lv_obj_clear_flag(g_paramCard, LV_OBJ_FLAG_HIDDEN);
      }
      break;
    case 4:
      if (sendCommand("B1")) { g_lightState = !g_lightState; g_status = "Licht getoggelt"; }
      break;
    case 5:
      if (sendCommand("K1")) { g_fanState = !g_fanState; g_status = "Luefter getoggelt"; }
      break;
    case 6:
      g_connectRequested = true;
      break;
    default: break;
  }
  syncUiText();
}

static void onDemoToggle(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
  lv_obj_t* cb = (lv_obj_t*)lv_event_get_target(e);
  g_demoEnabled = lv_obj_has_state(cb, LV_STATE_CHECKED);

  // Wenn Demo waehrend eines Demo-Laufs deaktiviert wird, zur Eingabeansicht zurueck.
  if (!g_demoEnabled && g_heatingActive && !g_connected) {
    g_heatingActive = false;
    setChartFullscreen(false);
    if (g_chartCard) lv_obj_add_flag(g_chartCard, LV_OBJ_FLAG_HIDDEN);
    if (g_paramCard) lv_obj_clear_flag(g_paramCard, LV_OBJ_FLAG_HIDDEN);
  }

  g_status = g_demoEnabled ? "Demo EIN" : "Demo AUS";
  applyStatusLed();
  syncUiText();
}

static void setStatusLedColor(uint8_t r, uint8_t g, uint8_t b) {
#if defined(STATUS_LED_R_PIN) && defined(STATUS_LED_G_PIN) && defined(STATUS_LED_B_PIN)
  // Diskrete RGB-LED ueber PWM, damit die Helligkeit sauber reduziert werden kann.
  uint8_t rLvl = (uint8_t)(((uint16_t)r * STATUS_LED_DIM) / 255U);
  uint8_t gLvl = (uint8_t)(((uint16_t)g * STATUS_LED_DIM) / 255U);
  uint8_t bLvl = (uint8_t)(((uint16_t)b * STATUS_LED_DIM) / 255U);
#if STATUS_LED_ACTIVE_LOW
  analogWrite(STATUS_LED_R_PIN, (uint8_t)(255U - rLvl));
  analogWrite(STATUS_LED_G_PIN, (uint8_t)(255U - gLvl));
  analogWrite(STATUS_LED_B_PIN, (uint8_t)(255U - bLvl));
#else
  analogWrite(STATUS_LED_R_PIN, rLvl);
  analogWrite(STATUS_LED_G_PIN, gLvl);
  analogWrite(STATUS_LED_B_PIN, bLvl);
#endif
#elif defined(RGB_BUILTIN)
  neopixelWrite(RGB_BUILTIN, r, g, b);
#elif defined(PIN_NEOPIXEL)
  neopixelWrite(PIN_NEOPIXEL, r, g, b);
#elif defined(LED_BUILTIN)
  // Fallback: einfache On/Off-LED ohne Farbe.
  digitalWrite(LED_BUILTIN, (r || g || b) ? HIGH : LOW);
#else
  (void)r; (void)g; (void)b;
#endif
}

static void applyStatusLed() {
  // Prioritaet: Demo > Verbunden > Nicht verbunden
  if (g_demoEnabled) {
    setStatusLedColor(48, 36, 0);   // schwach gelb, sichtbar
  } else if (g_connected) {
    setStatusLedColor(0, 44, 0);    // schwach gruen, sichtbar
  } else {
    setStatusLedColor(0, 0, 44);    // schwach blau, sichtbar
  }
}

// ============================================================================
// Hilfsfunktion: kompakter Action-Button
// ============================================================================
lv_obj_t* makeBtn(lv_obj_t* parent, const char* txt, int id,
                  int16_t w, int16_t h, lv_style_t* extra = nullptr) {
  lv_obj_t* btn = lv_button_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_add_style(btn, &sBtn, 0);
  if (extra) lv_obj_add_style(btn, extra, 0);
  lv_obj_add_event_cb(btn, onBtnEvent, LV_EVENT_CLICKED,
                      (void*)(uintptr_t)id);
  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, txt);
  lv_obj_add_style(lbl, &sTitle, 0);
  lv_obj_center(lbl);
  return btn;
}

// ============================================================================
// Dünne Trennlinie
// ============================================================================
static void addSeparator(lv_obj_t* parent, int16_t y) {
  lv_obj_t* line = lv_obj_create(parent);
  lv_obj_set_size(line, 280, 1);
  lv_obj_align(line, LV_ALIGN_TOP_MID, 0, y);
  lv_obj_set_style_bg_color(line, lv_color_hex(COL_BORDER), 0);
  lv_obj_set_style_bg_opa(line, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(line, 0, 0);
}

// ============================================================================
// UI aufbauen
// ============================================================================
void buildLvglUi() {
  lv_obj_t* scr = lv_screen_active();

  // ── Styles definieren ────────────────────────────────────────────────────
  lv_style_init(&sScreen);
  lv_style_set_bg_color(&sScreen, lv_color_hex(COL_BG));
  lv_style_set_bg_opa(&sScreen, LV_OPA_COVER);
  lv_obj_add_style(scr, &sScreen, 0);

  lv_style_init(&sCard);
  lv_style_set_bg_color(&sCard, lv_color_hex(COL_CARD));
  lv_style_set_bg_opa(&sCard, LV_OPA_COVER);
  lv_style_set_radius(&sCard, 10);
  lv_style_set_border_width(&sCard, 1);
  lv_style_set_border_color(&sCard, lv_color_hex(COL_BORDER));
  lv_style_set_pad_all(&sCard, 8);

  lv_style_init(&sTitle);
  lv_style_set_text_color(&sTitle, lv_color_hex(COL_TITLE));

  lv_style_init(&sBig);
  lv_style_set_text_color(&sBig, lv_color_hex(COL_VAL));

  lv_style_init(&sVal);
  lv_style_set_text_color(&sVal, lv_color_hex(COL_VAL2));

  lv_style_init(&sBtn);
  lv_style_set_radius(&sBtn, 8);
  lv_style_set_bg_color(&sBtn, lv_color_hex(COL_BTN));
  lv_style_set_bg_opa(&sBtn, LV_OPA_COVER);
  lv_style_set_border_color(&sBtn, lv_color_hex(COL_BORDER));
  lv_style_set_border_width(&sBtn, 1);

  lv_style_init(&sBtnDanger);
  lv_style_set_bg_color(&sBtnDanger, lv_color_hex(COL_BTN_RED));

  lv_style_init(&sBtnOk);
  lv_style_set_bg_color(&sBtnOk, lv_color_hex(COL_BTN_GREEN));

  lv_style_init(&sNumpad);
  lv_style_set_bg_color(&sNumpad, lv_color_hex(COL_KB_BG));
  lv_style_set_bg_opa(&sNumpad, LV_OPA_COVER);
  lv_style_set_radius(&sNumpad, 14);
  lv_style_set_border_color(&sNumpad, lv_color_hex(COL_ACCENT));
  lv_style_set_border_width(&sNumpad, 1);
  lv_style_set_pad_all(&sNumpad, 8);

  lv_style_init(&sNumBtn);
  lv_style_set_bg_color(&sNumBtn, lv_color_hex(COL_KB_BTN));
  lv_style_set_bg_opa(&sNumBtn, LV_OPA_COVER);
  lv_style_set_radius(&sNumBtn, 8);
  lv_style_set_border_width(&sNumBtn, 1);
  lv_style_set_border_color(&sNumBtn, lv_color_hex(COL_BORDER));

  lv_style_init(&sNumBtnDel);
  lv_style_set_bg_color(&sNumBtnDel, lv_color_hex(COL_KB_DEL));
  lv_style_set_bg_opa(&sNumBtnDel, LV_OPA_COVER);
  lv_style_set_radius(&sNumBtnDel, 8);
  lv_style_set_border_width(&sNumBtnDel, 1);
  lv_style_set_border_color(&sNumBtnDel, lv_color_hex(COL_BORDER));

  lv_style_init(&sNumBtnOk);
  lv_style_set_bg_color(&sNumBtnOk, lv_color_hex(COL_KB_OK));
  lv_style_set_bg_opa(&sNumBtnOk, LV_OPA_COVER);
  lv_style_set_radius(&sNumBtnOk, 8);
  lv_style_set_border_width(&sNumBtnOk, 1);
  lv_style_set_border_color(&sNumBtnOk, lv_color_hex(COL_BORDER));

  // ── Header ────────────────────────────────────────────────────────────────
  g_hdr = lv_obj_create(scr);
  lv_obj_set_size(g_hdr, 320, 36);
  lv_obj_align(g_hdr, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(g_hdr, lv_color_hex(0x0D1B2E), 0);
  lv_obj_set_style_bg_opa(g_hdr, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(g_hdr, 0, 0);
  lv_obj_set_style_radius(g_hdr, 0, 0);
  lv_obj_clear_flag(g_hdr, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* titleLbl = lv_label_create(g_hdr);
  lv_label_set_text(titleLbl, "BLEIron");
  lv_obj_add_style(titleLbl, &sTitle, 0);
  lv_obj_align(titleLbl, LV_ALIGN_LEFT_MID, 8, 0);

  g_lblBle = lv_label_create(g_hdr);
  lv_obj_add_style(g_lblBle, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblBle, lv_color_hex(0xF5B731), 0);
  lv_obj_align(g_lblBle, LV_ALIGN_RIGHT_MID, -8, 0);

  // ── Temperatur-Karte ──────────────────────────────────────────────────────
  g_tempCard = lv_obj_create(scr);
  lv_obj_set_size(g_tempCard, 300, 112);
  lv_obj_align(g_tempCard, LV_ALIGN_TOP_MID, 0, 42);
  lv_obj_add_style(g_tempCard, &sCard, 0);
  lv_obj_clear_flag(g_tempCard, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* tempTitle = lv_label_create(g_tempCard);
  lv_label_set_text(tempTitle, "IST-TEMPERATUR");
  lv_obj_add_style(tempTitle, &sTitle, 0);
  lv_obj_set_style_text_color(tempTitle, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(tempTitle, LV_ALIGN_TOP_MID, 0, 0);

  g_lblTemp = lv_label_create(g_tempCard);
  lv_label_set_text(g_lblTemp, "--.- C");
  lv_obj_add_style(g_lblTemp, &sBig, 0);
  lv_obj_set_style_transform_zoom(g_lblTemp, 430, 0);  // ~1.68x fuer bessere Lesbarkeit
  lv_obj_align(g_lblTemp, LV_ALIGN_CENTER, 0, 0);

  // ── Parameter-Karte ───────────────────────────────────────────────────────
  g_paramCard = lv_obj_create(scr);
  lv_obj_t* paramCard = g_paramCard;
  lv_obj_set_size(paramCard, 300, 154);
  lv_obj_align(paramCard, LV_ALIGN_TOP_MID, 0, 162);
  lv_obj_add_style(paramCard, &sCard, 0);
  lv_obj_clear_flag(paramCard, LV_OBJ_FLAG_SCROLLABLE);

  // Dauer-Zeile: Label | Eingabe | Einheit
  lv_obj_t* lbD = lv_label_create(paramCard);
  lv_label_set_text(lbD, "DAUER");
  lv_obj_add_style(lbD, &sTitle, 0);
  lv_obj_set_style_text_color(lbD, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(lbD, LV_ALIGN_TOP_LEFT, 0, 16);

  lv_obj_t* lbDs = lv_label_create(paramCard);
  lv_label_set_text(lbDs, "SEKUNDEN");
  lv_obj_add_style(lbDs, &sTitle, 0);
  lv_obj_set_style_text_color(lbDs, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(lbDs, LV_ALIGN_TOP_RIGHT, 0, 16);

  // Tippen => Numpad öffnen
  lv_obj_t* btnZendEdit = lv_button_create(paramCard);
  lv_obj_set_size(btnZendEdit, 98, 34);
  lv_obj_align(btnZendEdit, LV_ALIGN_TOP_LEFT, 92, 10);
  lv_obj_set_style_bg_color(btnZendEdit, lv_color_hex(0x0A1520), 0);
  lv_obj_set_style_bg_opa(btnZendEdit, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(btnZendEdit, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_border_width(btnZendEdit, 1, 0);
  lv_obj_set_style_radius(btnZendEdit, 6, 0);
  lv_obj_add_event_cb(btnZendEdit, onBtnEvent, LV_EVENT_CLICKED, (void*)(uintptr_t)0);

  g_lblZend = lv_label_create(btnZendEdit);
  lv_label_set_text(g_lblZend, "120");
  lv_obj_add_style(g_lblZend, &sBig, 0);
  lv_obj_center(g_lblZend);

  addSeparator(paramCard, 64);

  // Temperatur-Zeile: Label | Eingabe | Einheit
  lv_obj_t* lbT = lv_label_create(paramCard);
  lv_label_set_text(lbT, "ZIELTEMP.");
  lv_obj_add_style(lbT, &sTitle, 0);
  lv_obj_set_style_text_color(lbT, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(lbT, LV_ALIGN_TOP_LEFT, 0, 84);

  lv_obj_t* lbTc = lv_label_create(paramCard);
  lv_label_set_text(lbTc, "GRAD C");
  lv_obj_add_style(lbTc, &sTitle, 0);
  lv_obj_set_style_text_color(lbTc, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(lbTc, LV_ALIGN_TOP_RIGHT, 0, 84);

  lv_obj_t* btnTendEdit = lv_button_create(paramCard);
  lv_obj_set_size(btnTendEdit, 98, 34);
  lv_obj_align(btnTendEdit, LV_ALIGN_TOP_LEFT, 92, 78);
  lv_obj_set_style_bg_color(btnTendEdit, lv_color_hex(0x0A1520), 0);
  lv_obj_set_style_bg_opa(btnTendEdit, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(btnTendEdit, lv_color_hex(COL_ACCENT), 0);
  lv_obj_set_style_border_width(btnTendEdit, 1, 0);
  lv_obj_set_style_radius(btnTendEdit, 6, 0);
  lv_obj_add_event_cb(btnTendEdit, onBtnEvent, LV_EVENT_CLICKED, (void*)(uintptr_t)1);

  g_lblTend = lv_label_create(btnTendEdit);
  lv_label_set_text(g_lblTend, "180");
  lv_obj_add_style(g_lblTend, &sBig, 0);
  lv_obj_center(g_lblTend);

  g_lblStatus = lv_label_create(paramCard);
  lv_label_set_text(g_lblStatus, "Bereit");
  lv_obj_add_style(g_lblStatus, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblStatus, lv_color_hex(0x8AB0D0), 0);
  lv_obj_align(g_lblStatus, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  // ── Aktions-Karte ─────────────────────────────────────────────────────────
  g_actCard = lv_obj_create(scr);
  lv_obj_set_size(g_actCard, 300, 150);
  lv_obj_align(g_actCard, LV_ALIGN_BOTTOM_MID, 0, -6);
  lv_obj_add_style(g_actCard, &sCard, 0);
  lv_obj_clear_flag(g_actCard, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* btnCon = makeBtn(g_actCard, "CONNECT", 6, 130, 38);
  lv_obj_set_style_bg_color(btnCon, lv_color_hex(COL_BTN_ACT), 0);
  lv_obj_align(btnCon, LV_ALIGN_TOP_LEFT, 0, 2);

  lv_obj_t* btnStart = makeBtn(g_actCard, "START", 2, 130, 38, &sBtnOk);
  lv_obj_align(btnStart, LV_ALIGN_TOP_RIGHT, 0, 2);

  addSeparator(g_actCard, 52);

  lv_obj_t* btnStop = makeBtn(g_actCard, "STOP", 3, 88, 36, &sBtnDanger);
  lv_obj_align(btnStop, LV_ALIGN_TOP_LEFT, 0, 56);

  lv_obj_t* btnLicht = makeBtn(g_actCard, "LICHT", 4, 90, 36);
  lv_obj_align(btnLicht, LV_ALIGN_TOP_MID, 0, 56);

  lv_obj_t* btnFan = makeBtn(g_actCard, "LUEFTER", 5, 90, 36);
  lv_obj_align(btnFan, LV_ALIGN_TOP_RIGHT, 0, 56);

  g_lblMeta = lv_label_create(g_actCard);
  lv_obj_add_style(g_lblMeta, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblMeta, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblMeta, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  // ── Chart-Karte (gleiche Position wie paramCard, initial versteckt) ────────
  g_chartCard = lv_obj_create(scr);
  lv_obj_set_size(g_chartCard, 300, 154);
  lv_obj_align(g_chartCard, LV_ALIGN_TOP_MID, 0, 162);
  lv_obj_add_style(g_chartCard, &sCard, 0);
  lv_obj_clear_flag(g_chartCard, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(g_chartCard, LV_OBJ_FLAG_HIDDEN);

  // Chart-Titel
  lv_obj_t* chartTitle = lv_label_create(g_chartCard);
  lv_label_set_text(chartTitle, "HEIZKURVE");
  lv_obj_add_style(chartTitle, &sTitle, 0);
  lv_obj_set_style_text_color(chartTitle, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(chartTitle, LV_ALIGN_TOP_LEFT, 0, 0);

  g_btnChartFs = lv_button_create(g_chartCard);
  lv_obj_set_size(g_btnChartFs, 44, 24);
  lv_obj_align(g_btnChartFs, LV_ALIGN_TOP_RIGHT, 0, 0);
  lv_obj_add_style(g_btnChartFs, &sBtn, 0);
  lv_obj_add_event_cb(g_btnChartFs, onChartFullscreenToggle, LV_EVENT_CLICKED, nullptr);
  g_lblChartFsBtn = lv_label_create(g_btnChartFs);
  lv_label_set_text(g_lblChartFsBtn, "MAX");
  lv_obj_add_style(g_lblChartFsBtn, &sTitle, 0);
  lv_obj_center(g_lblChartFsBtn);

  // LVGL Chart
  g_chart = lv_chart_create(g_chartCard);
  lv_obj_set_size(g_chart, 268, 104);
  lv_obj_align(g_chart, LV_ALIGN_TOP_MID, 0, 18);
  lv_obj_clear_flag(g_chart, LV_OBJ_FLAG_SCROLLABLE);
  lv_chart_set_type(g_chart, LV_CHART_TYPE_LINE);
  lv_chart_set_update_mode(g_chart, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(g_chart, CHART_MAX_PTS);
  lv_chart_set_range(g_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 200);
  lv_chart_set_div_line_count(g_chart, 3, 5);
  lv_obj_set_style_bg_color(g_chart, lv_color_hex(0x060C16), 0);
  lv_obj_set_style_bg_opa(g_chart, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(g_chart, lv_color_hex(COL_BORDER), 0);
  lv_obj_set_style_border_width(g_chart, 1, 0);
  lv_obj_set_style_line_color(g_chart, lv_color_hex(COL_BORDER), LV_PART_MAIN);
  lv_obj_set_style_line_width(g_chart, 1, LV_PART_ITEMS);   // duenne Trendlinie
  lv_obj_set_style_width(g_chart, 0, LV_PART_INDICATOR);    // keine Punktmarker
  lv_obj_set_style_height(g_chart, 0, LV_PART_INDICATOR);   // keine Punktmarker
  lv_obj_set_style_pad_all(g_chart, 4, 0);

  g_chartSer = lv_chart_add_series(g_chart, lv_color_hex(COL_ACCENT),
                                   LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_all_values(g_chart, g_chartSer, LV_CHART_POINT_NONE);

  // Achsen-Beschriftung X: 0%, 50%, 100%
  g_lblChartXMin = lv_label_create(g_chartCard);
  lv_label_set_text(g_lblChartXMin, "");
  lv_obj_add_style(g_lblChartXMin, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartXMin, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartXMin, LV_ALIGN_BOTTOM_LEFT, 0, 0);

  g_lblChartXMid = lv_label_create(g_chartCard);
  lv_label_set_text(g_lblChartXMid, "60s");
  lv_obj_add_style(g_lblChartXMid, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartXMid, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartXMid, LV_ALIGN_BOTTOM_MID, 0, 0);

  g_lblChartXMax = lv_label_create(g_chartCard);
  lv_label_set_text(g_lblChartXMax, "120s");
  lv_obj_add_style(g_lblChartXMax, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartXMax, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartXMax, LV_ALIGN_BOTTOM_RIGHT, 0, 0);

  g_lblChartYMax = lv_label_create(g_chartCard);
  lv_label_set_text(g_lblChartYMax, "200 C");
  lv_obj_add_style(g_lblChartYMax, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartYMax, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartYMax, LV_ALIGN_TOP_LEFT, 0, 18);

  g_lblChartYMid = lv_label_create(g_chartCard);
  lv_label_set_text(g_lblChartYMid, "100 C");
  lv_obj_add_style(g_lblChartYMid, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartYMid, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartYMid, LV_ALIGN_TOP_LEFT, 0, 66);

  g_lblChartYMin = lv_label_create(g_chartCard);
  lv_label_set_text(g_lblChartYMin, "0 C");
  lv_obj_add_style(g_lblChartYMin, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartYMin, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartYMin, LV_ALIGN_TOP_LEFT, 0, 114);

  // Vollbild-Overlay fuer Chart
  g_chartFsOverlay = lv_obj_create(scr);
  lv_obj_set_size(g_chartFsOverlay, lv_pct(100), lv_pct(100));
  lv_obj_align(g_chartFsOverlay, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(g_chartFsOverlay, lv_color_hex(COL_BG), 0);
  lv_obj_set_style_bg_opa(g_chartFsOverlay, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(g_chartFsOverlay, 0, 0);
  lv_obj_set_style_radius(g_chartFsOverlay, 0, 0);
  lv_obj_set_style_pad_all(g_chartFsOverlay, 0, 0);
  lv_obj_clear_flag(g_chartFsOverlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(g_chartFsOverlay, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t* fsTitle = lv_label_create(g_chartFsOverlay);
  lv_label_set_text(fsTitle, "HEIZKURVE");
  lv_obj_add_style(fsTitle, &sTitle, 0);
  lv_obj_set_style_text_color(fsTitle, lv_color_hex(0x8AB0D0), 0);
  lv_obj_align(fsTitle, LV_ALIGN_TOP_LEFT, 10, 10);

  g_btnChartFsClose = lv_button_create(g_chartFsOverlay);
  lv_obj_set_size(g_btnChartFsClose, 52, 28);
  lv_obj_align(g_btnChartFsClose, LV_ALIGN_TOP_RIGHT, -10, 8);
  lv_obj_add_style(g_btnChartFsClose, &sBtn, 0);
  lv_obj_add_event_cb(g_btnChartFsClose, onChartFullscreenToggle, LV_EVENT_CLICKED, nullptr);
  g_lblChartFsClose = lv_label_create(g_btnChartFsClose);
  lv_label_set_text(g_lblChartFsClose, "MIN");
  lv_obj_add_style(g_lblChartFsClose, &sTitle, 0);
  lv_obj_center(g_lblChartFsClose);

  g_chartFs = lv_chart_create(g_chartFsOverlay);
  lv_obj_set_size(g_chartFs, 300, 238);
  lv_obj_align(g_chartFs, LV_ALIGN_TOP_MID, 2, 54);
  lv_obj_clear_flag(g_chartFs, LV_OBJ_FLAG_SCROLLABLE);
  lv_chart_set_type(g_chartFs, LV_CHART_TYPE_LINE);
  lv_chart_set_update_mode(g_chartFs, LV_CHART_UPDATE_MODE_SHIFT);
  lv_chart_set_point_count(g_chartFs, CHART_MAX_PTS);
  lv_chart_set_range(g_chartFs, LV_CHART_AXIS_PRIMARY_Y, 0, 200);
  lv_chart_set_div_line_count(g_chartFs, 5, 5);
  lv_obj_set_style_bg_color(g_chartFs, lv_color_hex(0x060C16), 0);
  lv_obj_set_style_bg_opa(g_chartFs, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(g_chartFs, lv_color_hex(COL_BORDER), 0);
  lv_obj_set_style_border_width(g_chartFs, 1, 0);
  lv_obj_set_style_line_color(g_chartFs, lv_color_hex(COL_BORDER), LV_PART_MAIN);
  lv_obj_set_style_line_width(g_chartFs, 2, LV_PART_ITEMS);
  lv_obj_set_style_width(g_chartFs, 0, LV_PART_INDICATOR);
  lv_obj_set_style_height(g_chartFs, 0, LV_PART_INDICATOR);
  lv_obj_set_style_pad_all(g_chartFs, 4, 0);

  g_chartFsSer = lv_chart_add_series(g_chartFs, lv_color_hex(COL_ACCENT),
                                     LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_all_values(g_chartFs, g_chartFsSer, LV_CHART_POINT_NONE);

  g_lblChartFsXMin = lv_label_create(g_chartFsOverlay);
  lv_label_set_text(g_lblChartFsXMin, "");
  lv_obj_add_style(g_lblChartFsXMin, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartFsXMin, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartFsXMin, LV_ALIGN_TOP_LEFT, 12, 296);

  g_lblChartFsXMid = lv_label_create(g_chartFsOverlay);
  lv_label_set_text(g_lblChartFsXMid, "60s");
  lv_obj_add_style(g_lblChartFsXMid, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartFsXMid, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartFsXMid, LV_ALIGN_TOP_MID, 0, 296);

  g_lblChartFsXMax = lv_label_create(g_chartFsOverlay);
  lv_label_set_text(g_lblChartFsXMax, "120s");
  lv_obj_add_style(g_lblChartFsXMax, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartFsXMax, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartFsXMax, LV_ALIGN_TOP_RIGHT, -12, 296);

  g_lblChartFsYMax = lv_label_create(g_chartFsOverlay);
  lv_label_set_text(g_lblChartFsYMax, "200 C");
  lv_obj_add_style(g_lblChartFsYMax, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartFsYMax, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartFsYMax, LV_ALIGN_TOP_LEFT, 10, 56);

  g_lblChartFsYMid = lv_label_create(g_chartFsOverlay);
  lv_label_set_text(g_lblChartFsYMid, "100 C");
  lv_obj_add_style(g_lblChartFsYMid, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartFsYMid, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartFsYMid, LV_ALIGN_TOP_LEFT, 10, 168);

  g_lblChartFsYMin = lv_label_create(g_chartFsOverlay);
  lv_label_set_text(g_lblChartFsYMin, "0 C");
  lv_obj_add_style(g_lblChartFsYMin, &sTitle, 0);
  lv_obj_set_style_text_color(g_lblChartFsYMin, lv_color_hex(0x6A8AAA), 0);
  lv_obj_align(g_lblChartFsYMin, LV_ALIGN_TOP_LEFT, 10, 282);

  // Demo-Checkbox: unten rechts in der Aktions-Karte (gleiche Hoehe wie Meta-Text)
  g_cbDemo = lv_checkbox_create(g_actCard);
  lv_checkbox_set_text(g_cbDemo, "DEMO");
  lv_obj_add_style(g_cbDemo, &sTitle, 0);
  lv_obj_set_style_text_color(g_cbDemo, lv_color_hex(0x8AB0D0), 0);
  lv_obj_align(g_cbDemo, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
  if (g_demoEnabled) {
    lv_obj_add_state(g_cbDemo, LV_STATE_CHECKED);
  } else {
    lv_obj_clear_state(g_cbDemo, LV_STATE_CHECKED);
  }
  lv_obj_add_event_cb(g_cbDemo, onDemoToggle, LV_EVENT_VALUE_CHANGED, nullptr);

  // Units-Labels oberhalb der Eingabefelder halten
  lv_obj_move_foreground(lbDs);
  lv_obj_move_foreground(lbTc);

  syncUiText();
  beginStartupAnimation(scr);
}

// ============================================================================
// BLE verbinden
// ============================================================================
bool connectToHeater() {
  if (!g_foundDevice) {
    g_status = "Scanne...";
    syncUiText();
    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCbs());
    scan->setInterval(100);
    scan->setWindow(99);
    scan->setActiveScan(true);
    scan->start(4, false);
  }
  if (!g_foundDevice) { g_status = "ESP32 nicht gefunden"; return false; }
  if (g_client && g_connected)  return true;

  g_status = "Verbinde...";
  syncUiText();
  g_client = BLEDevice::createClient();
  g_client->setClientCallbacks(new ClientCbs());
  if (!g_client->connect(g_foundDevice)) {
    g_status = "Connect fehlgeschl."; g_connected = false; return false;
  }
  BLERemoteService* svc = g_client->getService(kServiceUuid);
  if (!svc) { g_status = "Service fehlt"; g_client->disconnect(); return false; }
  g_rxChar = svc->getCharacteristic(kRxUuid);
  g_txChar = svc->getCharacteristic(kTxUuid);
  if (!g_rxChar || !g_txChar) {
    g_status = "Char fehlt"; g_client->disconnect(); return false;
  }
  if (g_txChar->canNotify()) g_txChar->registerForNotify(notifyCallback);
  g_status = "Verbunden"; g_connected = true;
  applyStatusLed();
  return true;
}

// ============================================================================
// setup / loop
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

#if STATUS_LED_RELEASE_GPIO1_FROM_SERIAL && ((STATUS_LED_R_PIN == 1) || (STATUS_LED_G_PIN == 1) || (STATUS_LED_B_PIN == 1))
  Serial.end();
#endif

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

#if defined(LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
#endif
#if defined(RGB_BUILTIN)
  pinMode(RGB_BUILTIN, OUTPUT);
#endif
#if defined(PIN_NEOPIXEL)
  pinMode(PIN_NEOPIXEL, OUTPUT);
#endif
#if defined(STATUS_LED_R_PIN) && defined(STATUS_LED_G_PIN) && defined(STATUS_LED_B_PIN)
  pinMode(STATUS_LED_R_PIN, OUTPUT);
  pinMode(STATUS_LED_G_PIN, OUTPUT);
  pinMode(STATUS_LED_B_PIN, OUTPUT);
#if STATUS_LED_ACTIVE_LOW
  digitalWrite(STATUS_LED_R_PIN, HIGH);
  digitalWrite(STATUS_LED_G_PIN, HIGH);
  digitalWrite(STATUS_LED_B_PIN, HIGH);
#else
  digitalWrite(STATUS_LED_R_PIN, LOW);
  digitalWrite(STATUS_LED_G_PIN, LOW);
  digitalWrite(STATUS_LED_B_PIN, LOW);
#endif
#endif

  ts.begin();
  ts.setRotation(ROTATION_INVERTED);

  BLEDevice::init("DisplayController");

  lv_init();
  g_display = lv_display_create(tft.width(), tft.height());
  lv_display_set_flush_cb(g_display, lvFlushCb);
  lv_display_set_buffers(g_display, g_buf1, nullptr, sizeof(g_buf1),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);

  g_inputDevice = lv_indev_create();
  lv_indev_set_type(g_inputDevice, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(g_inputDevice, lvTouchReadCb);

  buildLvglUi();
  applyStatusLed();
  g_lastLvTickMs = millis();
}

void loop() {
  uint32_t now   = millis();
  uint32_t delta = now - g_lastLvTickMs;
  g_lastLvTickMs = now;

  updateStartupAnimation();
  lv_tick_inc(delta);
  lv_timer_handler();

  if (g_connectRequested) {
    g_connectRequested = false;
    connectToHeater();
    syncUiText();
  }

  if (g_client && !g_client->isConnected() && g_connected) {
    g_connected = false; g_rxChar = nullptr; g_txChar = nullptr;
    g_status = "Getrennt";
    applyStatusLed();
    syncUiText();
  }

  // Demo-Isttemperatur (nur wenn nicht per BLE verbunden)
  if (g_demoEnabled && g_heatingActive && !g_connected) {
    float elapsedS = (millis() - g_heatStartMs) / 1000.0f;
    float progress = elapsedS / (float)g_zend;
    if (progress > 1.0f) progress = 1.0f;

    float ripple = (((int)elapsedS % 18) - 9) * 0.22f;  // leichte Messschwankung
    float ramp = g_demoAmbient + ((float)g_tend - g_demoAmbient) * progress;
    g_realTemp = ramp + ripple;
    if (g_realTemp < g_demoAmbient) g_realTemp = g_demoAmbient;
  }

  if (millis() - g_lastSyncMs >= 250) {
    g_lastSyncMs = millis();
    syncUiText();
  }

  // ── Chart-Datenpunkt alle 2s hinzufügen ──────────────────────────────────
  if (g_heatingActive && !isnan(g_realTemp)) {
    uint32_t nowMs = millis();
    if (nowMs - g_lastChartMs >= 2000) {
      g_lastChartMs = nowMs;
      uint32_t elapsedS = (nowMs - g_heatStartMs) / 1000;
      // Automatisch stoppen wenn Zeitfenster abgelaufen
      if (elapsedS > g_chartSpanSec) {
        g_heatingActive = false;
        setChartFullscreen(false);
      } else if ((g_chart && g_chartSer) || (g_chartFs && g_chartFsSer)) {
        if (g_chartPtCount < g_chartPointTarget) {
          g_chartTempBuf[g_chartPtCount] = g_realTemp;
          g_chartPtCount++;
        } else {
          for (uint16_t i = 1; i < g_chartPointTarget; ++i) {
            g_chartTempBuf[i - 1] = g_chartTempBuf[i];
          }
          g_chartTempBuf[g_chartPointTarget - 1] = g_realTemp;
        }
        refreshChartsFromBuffer();
      }
    }
  }

  delay(5);
}
