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
static constexpr uint32_t RING_SAMPLES = 4096;

// MUST be aligned to the ring size in BYTES (8 KB here)
static volatile uint16_t __attribute__((aligned(8192))) g_ring[RING_SAMPLES];

// -------------------- Globals --------------------
Adafruit_USBD_Audio usb;

//static volatile uint16_t g_ring[RING_SAMPLES];
static volatile uint32_t g_tail = 0;   // consumer index (samples)
static int g_dma_chan = -1;
static uint32_t g_underflows = 0;
static volatile uint32_t g_lastStreamMs = 0;  // updated in readCB when host pulls

// DMA head helper: where DMA is writing now (as sample index)
static inline uint32_t dma_head_samples() {
  io_rw_32 waddr = dma_channel_hw_addr(g_dma_chan)->write_addr;
  uintptr_t base  = reinterpret_cast<uintptr_t>(g_ring);
  uintptr_t bytes = static_cast<uintptr_t>(waddr) - base;
  return (bytes >> 1) & (RING_SAMPLES - 1);  // /2 for 16-bit, wrap
}

// -------------------- WS2812 LED --------------------
static void set_led(uint32_t color, uint8_t brightness) {
  strip.setBrightness(brightness);     // 0..255 scales output
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
  channel_config_set_ring(&cfg, true, 13);

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


// -------------------- USB audio callback --------------------
size_t readCB(uint8_t* data, size_t len, Adafruit_USBD_Audio& ref) {
  g_lastStreamMs = millis();

  uint16_t* out = reinterpret_cast<uint16_t*>(data);
  uint32_t samples_needed = len / 2;

  // Compute available samples
  uint32_t head = dma_head_samples();
  uint32_t available = (head >= g_tail) ? (head - g_tail)
                                        : (RING_SAMPLES - g_tail + head);

  if (available < samples_needed) {
    // Not enough: copy available then zero pad
    uint32_t i = 0;
    for (; i < available; ++i) {
      uint16_t s = g_ring[g_tail];
      g_tail = (g_tail + 1) & (RING_SAMPLES - 1);
      out[i] = (uint16_t)(s << 4); // 12-bit -> 16-bit
    }
    for (; i < samples_needed; ++i) out[i] = 0;
    g_underflows++;
    return len;
  }

  // Enough data
  for (uint32_t i = 0; i < samples_needed; ++i) {
    uint16_t s = g_ring[g_tail];
    g_tail = (g_tail + 1) & (RING_SAMPLES - 1);
    out[i] = (uint16_t)(s << 4);
  }
  return len;
}

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
