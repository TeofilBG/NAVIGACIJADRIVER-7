#include "mag_encoder.h"
#include <AiEsp32RotaryEncoder.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"

// ===== Defaults / config =====
namespace {
  constexpr int      kPosMin        = 0;
  constexpr int      kPosMax        = 180;
  constexpr int      kStartPos      = 90;

  // Median-3 (very low lag)
  constexpr int      kMedN          = 3;

  // Defaults (change at runtime via setters)
  constexpr int      kCountsPer180Def = 180;  // ~1°/count
  constexpr uint32_t kDebounceUsDef   = 700;  // 400–1200 typical
  constexpr int      kHystOutDef      = 1;    // trigger step
  constexpr int      kHystInDef       = 0;    // stickiness inside

  // Library ctor "steps per notch" (your examples used 4)
  constexpr int      kStepsPerNotch   = 4;
}

// ===== State =====
static int  g_pinA = -1;
static int  g_pinB = -1;

static volatile bool     g_invert       = false;
static volatile int      g_countsPer180 = kCountsPer180Def;
static volatile uint32_t g_debounceUs   = kDebounceUsDef;
static volatile int      g_hystOutDeg   = kHystOutDef;
static volatile int      g_hystInDeg    = kHystInDef;

static volatile uint32_t g_isrTicks     = 0;
static volatile uint32_t g_lastIrqUs    = 0;

static AiEsp32RotaryEncoder* g_enc = nullptr;

// Filters (foreground, degrees domain)
static int     s_hist[kMedN] = {kStartPos, kStartPos, kStartPos};
static uint8_t s_idx         = 0;
static int     s_last_report = kStartPos;

// ===== Helpers =====
static inline int clamp180(int v) {
  if (v < kPosMin) v = kPosMin;
  if (v > kPosMax) v = kPosMax;
  return v;
}
static inline int counts_to_deg(int32_t c, int cps180) {
  if (c < 0) c = 0;
  if (c > cps180) c = cps180;
  return (int)(( (int64_t)c * 180 + (cps180/2) ) / cps180);
}
static inline int32_t deg_to_counts(int deg, int cps180) {
  deg = clamp180(deg);
  return (int32_t)(( (int64_t)deg * cps180 + 90 ) / 180);
}
static inline uint16_t pos_to_raw12(int deg) {
  deg = clamp180(deg);
  return (uint16_t)(( (uint32_t)deg * 4095u ) / 180u);
}
static inline int median3(const int a[kMedN]) {
  int x=a[0], y=a[1], z=a[2];
  if (x>y){int t=x;x=y;y=t;} if (y>z){int t=y;y=z;z=t;} if (x>y){int t=x;x=y;y=t;}
  return y;
}
static void force_pullup(gpio_num_t pin) {
  // Clear any RTC config (harmless on non-RTC pins), then enforce strong pull-up
  rtc_gpio_deinit(pin);
  gpio_reset_pin(pin);
  gpio_set_direction(pin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
}
static void ensure_pullups_after_library() {
  // Some libs tweak the pin mode; enforce pull-ups again
  force_pullup((gpio_num_t)g_pinA);
  force_pullup((gpio_num_t)g_pinB);
  pinMode(g_pinA, INPUT_PULLUP);
  pinMode(g_pinB, INPUT_PULLUP);
}

// ===== ISR (A/B CHANGE handled inside the library) =====
static void IRAM_ATTR encoder_isr() {
  uint32_t now = micros();
  uint32_t db  = g_debounceUs;           // read volatile once
  if (now - g_lastIrqUs < db) return;    // debounce
  g_lastIrqUs = now;

  if (g_enc) g_enc->readEncoder_ISR();   // library's state update
  g_isrTicks++;
}

// ===== Public API =====
bool mag_begin(int sda_pin_asA, int scl_pin_asB, uint32_t /*i2c_freq_hz*/) {
  g_pinA = sda_pin_asA;
  g_pinB = scl_pin_asB;

  // Pre-begin pull-ups (so the first check shows 1/1)
  force_pullup((gpio_num_t)g_pinA);
  force_pullup((gpio_num_t)g_pinB);
  pinMode(g_pinA, INPUT_PULLUP);
  pinMode(g_pinB, INPUT_PULLUP);
  int a0 = gpio_get_level((gpio_num_t)g_pinA);
  int b0 = gpio_get_level((gpio_num_t)g_pinB);
  Serial.printf("[mag/lib] pre-begin pull-up check A=%d B=%d (expect 1/1)\n", a0, b0);

  // Create encoder exactly like your working snippet
  g_enc = new AiEsp32RotaryEncoder(g_pinA, g_pinB, /*btn*/-1, /*vcc*/-1, kStepsPerNotch);
  if (!g_enc) return false;

  g_enc->begin();
  g_enc->setup(encoder_isr);     // attaches CHANGE ISRs
  g_enc->setAcceleration(0);

  // Re-apply pull-ups AFTER library setup (critical)
  ensure_pullups_after_library();
  int a1 = gpio_get_level((gpio_num_t)g_pinA);
  int b1 = gpio_get_level((gpio_num_t)g_pinB);
  Serial.printf("[mag/lib] post-begin pull-up check A=%d B=%d (expect 1/1)\n", a1, b1);

  // Boundaries in COUNTS (0..g_countsPer180) so we can scale resolution later
  int cps180 = g_countsPer180;
  g_enc->setBoundaries(0, cps180, false);
  g_enc->setEncoderValue(deg_to_counts(kStartPos, cps180));

  // Reset filters
  s_last_report = kStartPos;
  for (int i=0;i<kMedN;i++) s_hist[i] = kStartPos;
  s_idx = 0;
  g_lastIrqUs = micros();

  Serial.printf("[mag/lib] using A=%d B=%d  steps/notch=%d  start=%d°  counts/180=%d  debounce=%luus  hyst=%d/%d\n",
                g_pinA, g_pinB, kStepsPerNotch, kStartPos, cps180,
                (unsigned long)g_debounceUs, g_hystOutDeg, g_hystInDeg);
  return true;
}

void mag_encoder_init() {
  if (!g_enc) return;
  int cps180 = g_countsPer180;
  g_enc->setBoundaries(0, cps180, false);
  g_enc->setEncoderValue(deg_to_counts(kStartPos, cps180));

  s_last_report = kStartPos;
  for (int i=0;i<kMedN;i++) s_hist[i] = kStartPos;
  s_idx = 0;

  Serial.printf("[mag/lib] Boundaries counts 0..%d  start=%d°\n", cps180, kStartPos);
}

int mag_get_encoder_position() {
  if (!g_enc) return kStartPos;

  // Keep fresh even if IRQs lag
  g_enc->readEncoder_ISR();

  // Lazy safety: if both lines somehow fell low, restore pull-ups
  static uint32_t lastCheck = 0;
  uint32_t now = millis();
  if (now - lastCheck > 500) {
    lastCheck = now;
    int a = gpio_get_level((gpio_num_t)g_pinA);
    int b = gpio_get_level((gpio_num_t)g_pinB);
    if (a == 0 && b == 0) ensure_pullups_after_library();
  }

  // Read counts from library, convert to degrees
  int cps180 = g_countsPer180;
  int32_t c  = g_enc->readEncoder();
  int deg    = counts_to_deg(c, cps180);

  if (g_invert) deg = 180 - deg;
  deg = clamp180(deg);

  // median-3
  s_hist[s_idx] = deg;
  s_idx = (s_idx + 1) % kMedN;
  int med = median3(s_hist);

  // Schmitt hysteresis around last reported value
  int outD = g_hystOutDeg;
  int inD  = g_hystInDeg;
  int delta = med - s_last_report;
  if (delta >= outD) {
    s_last_report += (delta - inD);
  } else if (delta <= -outD) {
    s_last_report += (delta + inD);
  }
  s_last_report = clamp180(s_last_report);
  return s_last_report;
}

void mag_reset_encoder() {
  if (!g_enc) return;
  int cps180 = g_countsPer180;
  g_enc->setEncoderValue(deg_to_counts(kStartPos, cps180));

  s_last_report = kStartPos;
  for (int i=0;i<kMedN;i++) s_hist[i] = kStartPos;
  s_idx = 0;
}

void mag_set_encoder_position(int position) {
  if (!g_enc) return;
  position = clamp180(position);
  int cps180 = g_countsPer180;
  g_enc->setEncoderValue(deg_to_counts(position, cps180));

  s_last_report = position;
  for (int i=0;i<kMedN;i++) s_hist[i] = position;
  s_idx = 0;
}

uint16_t mag_get_raw12() {
  return pos_to_raw12(mag_get_encoder_position());
}

float mag_get_angle_deg() {
  return (float)mag_get_encoder_position() * 2.0f;
}

void mag_set_zero_to_current() { /* no-op for mechanical */ }

float mag_get_angle_deg_zeroed() {
  return mag_get_angle_deg();
}

void mag_set_invert(bool invert) {
  g_invert = invert;
}

void mag_set_counts_per_180(int counts) {
  if (counts < 30) counts = 30;             // coarse but valid (~6°/count)
  if (counts > 16384) counts = 16384;       // sane upper bound

  if (!g_enc) { g_countsPer180 = counts; return; }

  // Preserve current angle while changing resolution
  int oldCps = g_countsPer180;
  int curCounts = g_enc->readEncoder();
  int curDeg    = counts_to_deg(curCounts, oldCps);

  g_countsPer180 = counts;
  g_enc->setBoundaries(0, g_countsPer180, false);
  g_enc->setEncoderValue(deg_to_counts(curDeg, g_countsPer180));

  // Reset filters to same angle
  s_last_report = curDeg;
  for (int i=0;i<kMedN;i++) s_hist[i] = curDeg;
  s_idx = 0;

  Serial.printf("[mag/lib] counts/180 set to %d (current %d°)\n", g_countsPer180, curDeg);
}

int mag_get_counts_per_180() {
  return g_countsPer180;
}

void mag_set_hysteresis(int out_deg, int in_deg) {
  if (out_deg < 0) out_deg = 0;
  if (in_deg  < 0) in_deg  = 0;
  if (in_deg > out_deg) in_deg = out_deg;

  g_hystOutDeg = out_deg;
  g_hystInDeg  = in_deg;
  Serial.printf("[mag/lib] hysteresis out/in = %d/%d deg\n", g_hystOutDeg, g_hystInDeg);
}

void mag_set_debounce_us(uint32_t us) {
  if (us < 200)  us = 200;      // avoid double-counts
  if (us > 5000) us = 5000;     // avoid missing fast turns too much
  g_debounceUs = us;
  Serial.printf("[mag/lib] debounce = %lu us\n", (unsigned long)g_debounceUs);
}

extern "C" uint32_t mag_debug_isr_ticks() { return g_isrTicks; }
