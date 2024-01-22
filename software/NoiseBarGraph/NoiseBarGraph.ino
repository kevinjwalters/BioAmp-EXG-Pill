// Show raw envelope vs filtered envelope as a pair of log bar charts
// https://github.com/upsidedownlabs/BioAmp-EXG-Pill

// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2021 Upside Down Labs - contact@upsidedownlabs.tech
// Copyright (c) 2024 Kevin J. Walters

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <inttypes.h>
#include <math.h>


#if defined(ARDUINO_UNOR4_WIFI)
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"

ArduinoLEDMatrix matrix;

#define DISPLAY_WIDTH 12
#define DISPLAY_HEIGHT 8
#define DISPLAY_PIXELS (DISPLAY_WIDTH * DISPLAY_HEIGHT)

byte frame[DISPLAY_HEIGHT][DISPLAY_WIDTH] = {
  { 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0 },
  { 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0 },
  { 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1 },
  { 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0 },
  { 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0 },
  { 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0 },
  { 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0 }
};
#endif

// 150 samples (300ms at 500sps) oscillates less as mains interference
// which survives the filter is 15 cycles at 60Hz or 18 cycles at 50Hz
#define BUFFER_SIZE 150
#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN A0
// #define EMG_MIN 2
// #define EMG_MAX 10
#define EMG_MIN 0
#define EMG_MAX 1023
#define EMG_MAX_LOG 6.9314f

static constexpr float half_bar_chart_log_scale = DISPLAY_PIXELS / 2 / 6.9314f; // log(1024.0) = 6.931471805599453

class SimpleCircularBuffer {
 public:
  SimpleCircularBuffer(size_t elements) : elements(elements) {
    circular_buffer = new int[elements];
  }
  ~SimpleCircularBuffer(void) { delete[] circular_buffer; }
  
  // Envelop detection algorithm
  float storeAndGetEnvelop(int abs_emg){
    sum -= circular_buffer[data_index];
    sum += abs_emg;
    circular_buffer[data_index] = abs_emg;
    data_index = (data_index + 1) % elements;
    return (float(sum) / BUFFER_SIZE) * 2.0f;
  } 

 private:
  int *circular_buffer;
  size_t elements;
  int32_t sum;
  int data_index;
};

SimpleCircularBuffer filtered_buffer(BUFFER_SIZE);
SimpleCircularBuffer unfiltered_buffer(BUFFER_SIZE);


inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#if defined(ARDUINO_UNOR4_WIFI)
static void bar_chart(int value, int height, int y_offset, bool render) {
  int max_value = DISPLAY_WIDTH * height;
  bool done = false;

  if (value >= max_value) {
    memset(frame[y_offset], 1, max_value);
    done = true;
  } else {
    memset(frame[y_offset], 0, max_value);
    if (value <= 0) {
      done = true;
    }
  }

  if (!done) {
    // value is between 1 and 95 (for full size bar chart) inclusive
    int cols = value / height;
    int extra_pixels = value % height;
    int mid_pos = height / 2;
    int y = mid_pos;
    int row_offset = 0;
    for (int row = 0; row < height; row++) {
      int bar_pixels = cols + ((extra_pixels > 0) ? 1 : 0);
      memset(frame[y + y_offset], 1, bar_pixels);

      row_offset = 0 - row_offset;
      if (row_offset >= 0) { ++row_offset; }
      y = mid_pos - row_offset;

      if (extra_pixels > 0) {
        --extra_pixels;
      }
    }
  }

  if (render) {
    matrix.renderBitmap(frame, DISPLAY_HEIGHT, DISPLAY_WIDTH);
  }
}
#endif


void setup() {
  // Serial connection begin
  Serial.begin(BAUD_RATE);
#if defined(ARDUINO_UNOR4_WIFI)
  matrix.begin();
  matrix.renderBitmap(frame, DISPLAY_HEIGHT, DISPLAY_WIDTH);
  delay(3 * 1000);  // 3 second pause
#endif
}

void loop() {
  
  // Calculate elapsed time
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;

  // Sample and get envelop
  if(timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    int sensor_value = analogRead(INPUT_PIN);
    float envelop_unf = unfiltered_buffer.storeAndGetEnvelop(sensor_value);

    int signal = (int)EMGFilter((float)sensor_value);
    float envelop_fil = filtered_buffer.storeAndGetEnvelop(abs(signal));

#if defined(ARDUINO_UNOR4_WIFI)
    int bar_chart_value_unf = constrain((int)roundf(mapf(logf((float)envelop_unf + 1.0f),
                                                         EMG_MIN, EMG_MAX_LOG,
                                                         -0.499f, DISPLAY_PIXELS / 2 + 0.499f)),
                                    0, DISPLAY_PIXELS / 2);
    int bar_chart_value_fil = constrain((int)roundf(mapf(logf(envelop_fil + 1.0f),
                                                         EMG_MIN, EMG_MAX_LOG,
                                                         -0.499f, DISPLAY_PIXELS / 2 + 0.499f)),
                                    0, DISPLAY_PIXELS / 2);
    bar_chart(bar_chart_value_unf, 4, 0, false);
    bar_chart(bar_chart_value_fil, 4, 4, true);
#endif
    Serial.print(sensor_value);
    Serial.print(",");
    Serial.print(envelop_unf);
    Serial.print(",");
    Serial.print(signal);
    Serial.print(",");
    Serial.println(envelop_fil);
  }
}


// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: 
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

