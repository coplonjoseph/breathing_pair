#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include "config.h"
#include "AdafruitIO_WiFi.h"
#ifdef __AVR__
  #include <avr/power.h>
#endif

/* Neopixel defines */
#define PIN_NEOPIXEL 14
#define NUM_LEDS 16
#define BRIGHTNESS 255

/* Microphone defines */
#define PIN_MIC 0
#define SAMPLEWINDOW 25

#if COOL_BOI==1
#define RX_FEED "breaths.js_breath"
#define TX_FEED "breaths.js_breath"
#else
#define RX_FEED "breaths.as_breath"
#define TX_FEED "breaths.js_breath"
#endif

/* Circular buffer for storing breath values */
#define IO_BUFSIZE 8 // Buffer stored in Adafruit IO
#define RX_BUFSIZE IO_BUFSIZE * 8
uint8_t rx_buf[RX_BUFSIZE];
uint8_t rx_head = 0;
uint8_t rx_tail = 0;
uint8_t tx_buf[IO_BUFSIZE];
uint8_t tx_head = 0;
#define TX_PERIOD 4000/IO_BUFSIZE
unsigned long tx_millis = 0;

uint16_t current_pixel = 0;

int rx_val = 0;
int tx_val = 0;

uint8_t mic_val;

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN_NEOPIXEL, NEO_GRBW + NEO_KHZ800);

AdafruitIO_Feed *rx_breath = io.feed(RX_FEED);
AdafruitIO_Feed *tx_breath = io.feed(TX_FEED);

void setup() {
  
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  Serial.begin(115200);

  Serial.print("Connecting to Adafruit IO");
  io.connect();
  rx_breath -> onMessage(handleMessage);

  uint16_t i = 0;

  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    strip.clear();
    strip.setPixelColor(i, strip.Color(0, 0, 0, 75));
    strip.show();
    i = (i+2) % strip.numPixels();    
    delay(100);
  }

  Serial.println();
  Serial.println(io.statusText());
  rx_breath -> get();
  
  tx_millis = millis();
}

void loop() {
  io.run();

  // Once per tx period
  if (millis() - tx_millis > TX_PERIOD) {
    tx_millis = millis();

    mic_val = (sample_mic()/2) + (mic_val/4); // add a discrete low pass filter
    tx_buf[tx_head++] = mic_val;
    tx_head &= IO_BUFSIZE - 1;

    Serial.print("Low pass val: ");
    Serial.println(mic_val);
  
    if (tx_head == 0) {
      uint32_t tx_cat = 0;
      for (uint8_t i=0; i<IO_BUFSIZE; i++) {
        tx_cat |= (tx_buf[i] & 0x0F) << (i*4);
      }
      tx_breath->save(tx_cat);
      Serial.print("Pushed: ");
      Serial.println(tx_cat,HEX);
    }    

    fetch_value();
  }
  
  tx_val = tx_buf[tx_head-1];
  
  #if COOL_BOI==1
  strip.setPixelColor(current_pixel, strip.Color(rx_val * 17, (rx_val + tx_val)*4, tx_val * 17));
  #else
  strip.setPixelColor(current_pixel, strip.Color(tx_val * 17, (rx_val + tx_val)*4, rx_val * 17));
  #endif
  strip.show();
  current_pixel = (current_pixel+1) % strip.numPixels();    
    
  delay(50);
}

/* Receive messages from feed and place into rx buffer */
void handleMessage(AdafruitIO_Data *data) {
  uint32_t data_val = data->toUnsignedLong();
  for (int i=0; i<IO_BUFSIZE; i++)
  {
    rx_buf[rx_head++] = (data_val >> (i * 4)) & 0x0F; // Put value in buffer
    rx_head &= RX_BUFSIZE-1; // Wrap head if necessary
  }
  Serial.print("recieved <- Breath: ");
  Serial.println(data_val, HEX);
}

/* Fetch value from rx buffer */
void fetch_value() {
  //if (((rx_tail+1) & (RX_BUFSIZE-1)) != rx_head) { // Prevent overflow
  if (rx_tail != rx_head) {
    rx_val = rx_buf[rx_tail++]; // Fetch value
    rx_tail &= RX_BUFSIZE-1; // Wrap tail index if necessary

    Serial.print("Fetched rx val: ");
    Serial.println(rx_val);
  }
}

/* Retrieve a sample from the microphone */
uint8_t sample_mic() {
  unsigned long startMillis = millis();
  unsigned int peakToPeak = 0;

  unsigned int signalMax = 0;
  unsigned int signalMin = 0;

  unsigned int sample = 0;

  uint8_t val;

  while (millis() - startMillis < SAMPLEWINDOW)
  {
    sample = analogRead(PIN_MIC);
    if (sample < 1024)
    {
      if (sample > signalMax)
      {
        signalMax = sample;
      }
      else if (sample < signalMax)
      {
        signalMin = sample;
      }
    }
  }
  peakToPeak = signalMax - signalMin;
  val = (uint8_t) map(peakToPeak, 0, 1023, 0, 15);
  Serial.print("mic val: ");
  Serial.println(val);

  return val;
}
