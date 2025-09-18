// RP2040 USB Microphone @ 384000 Hz, 16-bit, mono
// - DMA from ADC1 (GPIO27/A1) into a ring buffer
// - TinyUSB Audio (Phil's Audio branch) streams to host
// - WS2812 status LED: breathing (idle), solid (streaming), fast red blink (underflow)

#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include <Adafruit_NeoPixel.h>

extern "C" {
  #include "hardware/adc.h"
  #include "hardware/dma.h"
  #include "hardware/irq.h"
}

// -------------------- User config --------------------
static constexpr uint32_t SAMPLE_RATE_HZ  = 384000;
static constexpr uint8_t  CHANNELS        = 1;
static constexpr uint8_t  SAMPLE_BITS     = 16;   // don't call this BITS_PER_SAMPLE (macro clash)

// WS2812 (NeoPixel)
#define WS2812_PIN    16    // <-- set this to your LED data pin
#define WS2812_COUNT  1
Adafruit_NeoPixel strip(WS2812_COUNT, WS2812_PIN, NEO_GRB + NEO_KHZ800);

// Colors
static const uint32_t COL_IDLE   = Adafruit_NeoPixel::Color(255, 180, 0);   // warm yellow (breathing)
static const uint32_t COL_STREAM = Adafruit_NeoPixel::Color(0, 255, 0);     // solid green
static const uint32_t COL_ERROR  = Adafruit_NeoPixel::Color(255, 0, 0);     // red (blink on underflow)

// ADC input pin (RP2040 ADC1 = GPIO27 = A1)
static constexpr uint8_t  ADC_GPIO = 27;  // A1 -> ADC1

// Ring buffer sizing (samples, power-of-two)
//static constexpr uint32_t RING_SAMPLES = 8192; // 16 KB (8192 * 2B) ≈ 21 ms @ 384k/16/mono
// 8 KB ring window (power-of-two bytes). 4096 samples × 2 bytes = 8192 bytes.
static constexpr uint32_t RING_SAMPLES = 8192;

// MUST be aligned to the ring size in BYTES (8 KB here)
static volatile uint16_t __attribute__((aligned(16384))) g_ring[RING_SAMPLES];

// -------------------- Globals --------------------
Adafruit_USBD_Audio usb;

//static volatile uint16_t g_ring[RING_SAMPLES];
static volatile uint32_t g_tail = 0;   // consumer index (samples)
static int g_dma_chan = -1;
static uint32_t g_underflows = 0;
static volatile uint32_t g_lastStreamMs = 0;  // updated in readCB when host pulls
static volatile int32_t g_dc_offset_12 = 2048; // 12-bit mid-code is ~2048 if your front-end is biased at Vref/2

// DMA head helper: where DMA is writing now (as sample index)
static inline uint32_t dma_head_samples() {
  io_rw_32 waddr = dma_channel_hw_addr(g_dma_chan)->write_addr;
  uintptr_t base  = reinterpret_cast<uintptr_t>(g_ring);
  uintptr_t bytes = static_cast<uintptr_t>(waddr) - base;
  return (bytes >> 1) & (RING_SAMPLES - 1);  // /2 for 16-bit, wrap
}

// -------------------- WS2812 LED --------------------
static void set_led(uint32_t color, uint8_t brightness) {
  static uint32_t lastColor = 0xFFFFFFFF;
  static uint8_t  lastBright = 255;
  static uint32_t lastShow = 0;

  uint32_t now = millis();
  bool changed = (color != lastColor) || (brightness != lastBright);
  if (!changed && (now - lastShow) < 20) return;  // ≤50 Hz updates

  lastColor = color;
  lastBright = brightness;
  lastShow = now;

  strip.setBrightness(brightness);
  strip.setPixelColor(0, color);
  strip.show();
}

static void led_task() {
  const uint32_t now = millis();
  static uint32_t lastBlink = 0;
  static bool blink = false;

  const bool streaming_recent = (now - g_lastStreamMs) < 200; // pulled in last 200 ms
  const bool had_underflow    = (g_underflows > 0);

  if (had_underflow) {
    // Fast red blink
    if (now - lastBlink >= 100) {
      lastBlink = now;
      blink = !blink;
      set_led(COL_ERROR, blink ? 255 : 0);
    }
    return;
  }

  if (streaming_recent) {
    // Solid green while streaming
    set_led(COL_STREAM, 255);
  } else {
    // Breathing yellow (2.5 s triangle)
    float t = (now % 2500) / 2500.0f;                // 0..1
    float tri = t < 0.5f ? (t * 2.0f) : (2.0f - t * 2.0f);
    uint8_t v = (uint8_t)(tri * 255.0f);
    set_led(COL_IDLE, v);
  }
}

// -------------------- ADC + DMA init --------------------
static void adc_dma_begin() {
  // --- ADC ---
  adc_init();
  adc_gpio_init(27);     // GPIO27 / ADC1
  adc_select_input(1);

  // FIFO for DMA: request when >=1 sample, no 8-bit shift
  adc_fifo_setup(true, true, 1, false, false);

  // Target ~384 kS/s. ADC needs ~96 ADC clocks per sample.
  // clkdiv = f_adcclk / (Fs * 96). RP2040 accepts 16.8 fixed-point here.
  float clkdiv = 48000000.0f / (384000.0f * 96.0f);  // ≈ 1.302
  adc_set_clkdiv(clkdiv);

  // --- DMA to ring ---
  g_underflows = 0;
  g_tail = 0;

  int ch = dma_claim_unused_channel(true);
  g_dma_chan = ch;

  dma_channel_config cfg = dma_channel_get_default_config(ch);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);   // 16-bit samples
  channel_config_set_read_increment(&cfg, false);             // from FIFO
  channel_config_set_write_increment(&cfg, true);             // into buffer
  channel_config_set_dreq(&cfg, DREQ_ADC);                    // paced by ADC

  // 8 KB ring (2^13). Buffer is aligned(8192), so this is safe.
  channel_config_set_ring(&cfg, true, 14);

  dma_channel_configure(
      ch,
      &cfg,
      (void*)g_ring,              // write dest (8 KB-aligned)
      (const void*)&adc_hw->fifo, // read src
      0xFFFFFFFF,                 // run “forever”, wrapping in ring
      false                       // don't start yet
  );

  // Start DMA, then free-running ADC
  dma_start_channel_mask(1u << ch);
  adc_run(true);
}

static uint16_t measure_dc_offset(uint32_t n = 4096) {
  const uint32_t mask = RING_SAMPLES - 1;
  uint32_t head = dma_head_samples();
  uint32_t start = (head - n) & mask;
  uint64_t acc = 0;
  for (uint32_t i = 0; i < n; ++i) acc += g_ring[(start + i) & mask];
  return (uint16_t)(acc / n);
}


// -------------------- USB audio callback --------------------
size_t readCB(uint8_t* data, size_t len, Adafruit_USBD_Audio& /*ref*/) {
  g_lastStreamMs = millis();

  int16_t* out = reinterpret_cast<int16_t*>(data);   // SIGNED PCM16
  const uint32_t samples_needed = len / 2;           // 384 @ 384 kHz
  const uint32_t mask = RING_SAMPLES - 1;

  // Fixed latency behind DMA writer (keeps frames stable)
  const uint32_t LAT_MS = 4;
  const uint32_t LAT = LAT_MS * (SAMPLE_RATE_HZ / 1000u);  // 1536 @ 384k
  const uint32_t head = dma_head_samples();
  uint32_t start = (head - LAT - samples_needed) & mask;

  for (uint32_t i = 0; i < samples_needed; ++i) {
    uint16_t s12 = g_ring[(start + i) & mask];            // 0..4095
    int32_t v = ((int32_t)s12 - g_dc_offset_12) << 4;     // center, 12→16
    if (v >  32767) v =  32767;                           // saturate (rare)
    if (v < -32768) v = -32768;
    out[i] = (int16_t)v;                                  // signed PCM
  }

  // For stats/LED only
  g_tail = (start + samples_needed) & mask;
  return len;
}

// size_t readCB(uint8_t* data, size_t len, Adafruit_USBD_Audio& /*ref*/) {
//   g_lastStreamMs = millis();

//   int16_t* out = reinterpret_cast<int16_t*>(data);   // signed PCM16
//   const uint32_t N    = len / 2;                     // 384 @ 384 kHz
//   const uint32_t mask = RING_SAMPLES - 1;
//   const uint32_t head = dma_head_samples();

//   // Target latency and small safety margin
//   constexpr uint32_t LAT_MS = 6;                                   // try 6 ms
//   constexpr uint32_t LAT    = LAT_MS * (SAMPLE_RATE_HZ / 1000u);   // 2304
//   constexpr uint32_t GUARD  = 8;                                   // ~20 µs

//   // Persistent read pointer and sync flag
//   static bool     synced   = false;
//   static uint32_t read_pos = 0;

//   auto dist = [&](uint32_t a, uint32_t b)->uint32_t { return (b - a) & mask; };

//   // Initial sync once: sit LAT+N behind the DMA writer
//   if (!synced) {
//     read_pos = (head - LAT - N) & mask;
//     synced = true;
//   }

//   // How much is currently buffered ahead of us
//   uint32_t available = dist(read_pos, head);

//   // Soft PLL: keep available near target = LAT+N by nudging ±1 sample
//   const int32_t target = (int32_t)LAT + (int32_t)N;
//   int32_t d = (int32_t)available;
//   if (d > (int32_t)(mask/2)) d -= (int32_t)(mask+1);    // wrap to signed
//   int32_t err = d - target;
//   int32_t adjust = (err >  32) ? +1 : (err < -32) ? -1 : 0;

//   uint32_t start = (read_pos + (uint32_t)adjust) & mask;

//   // Keep a tiny guard so we never read the word DMA is about to write
//   uint32_t safe_avail = dist(start, head);
//   uint32_t safe_to_copy = (safe_avail > GUARD) ? (safe_avail - GUARD) : 0;

//   // We will ALWAYS output N samples. If short, we duplicate the last sample to fill.
//   uint32_t to_copy = (safe_to_copy >= N) ? N : safe_to_copy;
//   uint32_t i = 0;

//   // Copy what is safely available
//   int16_t last = 0;
//   for (; i < to_copy; ++i) {
//     uint16_t s12 = g_ring[(start + i) & mask];          // 0..4095
//     int32_t v = ((int32_t)s12 - g_dc_offset_12) << 4;   // center, 12→16
//     if (v >  32767) v =  32767;
//     if (v < -32768) v = -32768;
//     out[i] = last = (int16_t)v;
//   }

//   // If we were short, fill the remainder by repeating the last sample (no zeros)
//   for (; i < N; ++i) out[i] = last;

//   // Advance by EXACTLY what we consumed from the ring (not the padded part)
//   read_pos = (start + to_copy) & mask;
//   g_tail   = read_pos;     // for your LED/debug

//   // (Optional) only count as underflow if we had to pad a lot, e.g. to_copy < N-8
//   // if (to_copy + 8 < N) g_underflows++;

//   return len;
// }

// -------------------- Arduino setup/loop --------------------
void setup() {
  // --- WS2812 init ---
  strip.begin();
  strip.clear();
  strip.setBrightness(0);
  // strip.show(); // optional

  // --- Force a clean USB identity & re-enumeration BEFORE host binds ---
  // 1) Temporarily detach from the bus so the host forgets current instance
  TinyUSBDevice.detach();
  delay(20);

  // 2) Set fresh VID/PID/strings (must be before we attach back)
  TinyUSBDevice.setID(0x239A, 0xCB41);                       // NEW PID
  TinyUSBDevice.setProductDescriptor("RP2040 384k Mono Mic");
  TinyUSBDevice.setManufacturerDescriptor("Waveshare");
  TinyUSBDevice.setSerialDescriptor("rp2040-384k-uniq-001"); // NEW serial

  // 3) Register the audio interface with the desired format
  usb.setReadCallback(readCB);
  usb.begin(SAMPLE_RATE_HZ, CHANNELS, SAMPLE_BITS);          // 384000, 1, 16

  // 4) Attach back to the bus = host sees a brand-new device (PID/Serial)
  TinyUSBDevice.attach();

  // --- Wait until Windows actually mounts us before starting ADC/DMA ---
  uint32_t t0 = millis();
  while (!TinyUSBDevice.mounted() && (millis() - t0 < 3000)) {
    delay(10);
    yield();  // lets TinyUSB run
  }

  // Start ADC+DMA only after mount (enumeration stable)
  adc_dma_begin();

  // in setup(), after adc_dma_begin();
  delay(10);                               // let the ring fill a bit
  g_dc_offset_12 = measure_dc_offset();    // replaces the 2048 guess
}

void loop() {
  if (TinyUSBDevice.mounted()) {
    led_task();               // this may call strip.show()
  }

  // Light debug once per second
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 1000) {
    last = now;
    uint32_t head = dma_head_samples();
    uint32_t avail = (head >= g_tail) ? (head - g_tail)
                                      : (RING_SAMPLES - g_tail + head);
    Serial.printf("avail=%lu samp, underflows=%lu\n",
                  (unsigned long)avail, (unsigned long)g_underflows);
  }
}
