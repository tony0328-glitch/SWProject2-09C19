// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficient to convert duration to distance

// ===== Median filter setting =====
// For the assignment, test MEDIAN_N = 3, 10, and 30.
#define MEDIAN_N 30

// global variables
unsigned long last_sampling_time = 0; // unit: msec
float samples[MEDIAN_N];              // stores the most recent N raw samples
int sample_index = 0;                 // next position to overwrite
int sample_count = 0;                 // number of valid slots currently filled

float USS_measure(int TRIG, int ECHO);
float median_filter(float new_sample);

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;
  float dist_median;

  // wait until next sampling time
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a raw distance reading from the ultrasonic sensor
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // Challenge: use ONLY the median filter.
  // Do NOT replace invalid measurements with the previous valid value.
  dist_median = median_filter(dist_raw);

  // output values to Serial Plotter
  Serial.print("Min:");
  Serial.print(_DIST_MIN);
  Serial.print(",raw:");
  Serial.print(min(dist_raw, (float)(_DIST_MAX + 100)));
  Serial.print(",median:");
  Serial.print(min(dist_median, (float)(_DIST_MAX + 100)));
  Serial.print(",Max:");
  Serial.print(_DIST_MAX);
  Serial.println("");

  // LED indicates whether the RAW value is inside the desired range
  if ((dist_raw < _DIST_MIN) || (dist_raw > _DIST_MAX))
    digitalWrite(PIN_LED, HIGH); // LED OFF
  else
    digitalWrite(PIN_LED, LOW);  // LED ON

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// Median filter: keep the most recent MEDIAN_N raw samples,
// sort a temporary copy, and return the middle value.
float median_filter(float new_sample) {
  // store newest raw sample in circular buffer
  samples[sample_index] = new_sample;
  sample_index = (sample_index + 1) % MEDIAN_N;

  if (sample_count < MEDIAN_N)
    sample_count++;

  // copy current samples so the original circular buffer is not reordered
  float temp[MEDIAN_N];
  for (int i = 0; i < sample_count; i++) {
    temp[i] = samples[i];
  }

  // simple ascending sort (bubble sort)
  for (int i = 0; i < sample_count - 1; i++) {
    for (int j = 0; j < sample_count - 1 - i; j++) {
      if (temp[j] > temp[j + 1]) {
        float t = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = t;
      }
    }
  }

  // odd number of samples: return the center value
  if (sample_count % 2 == 1) {
    return temp[sample_count / 2];
  }

  // even number of samples: average the two center values
  return (temp[sample_count / 2 - 1] + temp[sample_count / 2]) / 2.0;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
