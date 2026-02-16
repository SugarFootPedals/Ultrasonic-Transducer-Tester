/*
  Ultrasonic Transducer Resonant Frequency Scanner with Band Selection Menu

  Option 1: User selects a frequency range at startup using the START button.
  - Short press (tap and release): cycles bands
  - Long press (hold > 700ms): selects band for scan

  - Two-pass sweep: Coarse and Fine
  - OLED display progress/results
  - ACS712 Current measurement

  Bands:
    - 20kHz to 46kHz
    - 46kHz to 65kHz
    - 65kHz to 85kHz

    Working!
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <ACS712.h>

// Debug flag
#define DEBUG 0

// OLED display parameters for 128x64 SSD1306 I2C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pins (adjust to match board)
const int PWM_PIN = D6;
const int I_SENSE_PIN = A0;
const int START_PIN = D1;

// ADC / sensor calibration constants
#define ADC_MAX            4095.0f
#define VREF               3.3f
#define ACS_SENSITIVITY    0.066f

// Sweep parameters (coarse/fine)
const unsigned int FREQ_STEP = 50;
const unsigned int SAMPLES_PER_FREQ = 20;
const unsigned int STABILIZE_DELAY = 15;

const unsigned int FINE_WINDOW = 1000;
const unsigned int FINE_STEP = 10;
const unsigned int FINE_SAMPLES_PER_FREQ = 20;
const int MAX_FINE_TARGETS = 3;

// --- Band Selection Section ---
typedef struct {
  unsigned int start;
  unsigned int end;
  const char* label;
} ScanBand;

ScanBand bands[] = {
  {20000, 46000, "20-46kHz"},
  {46000, 65000, "46-65kHz"},
  {65000, 85000, "65-85kHz"}
};
const int numBands = sizeof(bands) / sizeof(bands[0]);
int bandSelected = 0; // Selected band index
unsigned int START_FREQ;
unsigned int END_FREQ;

// Variables to track resonance
unsigned int resonantFreq = 0;
float maxAmplitude = 0;
float currAmplitude = 0;
float resonantIpk = 0;
float resonantIpp = 0;
bool testRunning = false;
bool testComplete = false;

// Data storage for plotting
#define MAX_DATA_POINTS 70
unsigned int freqs[MAX_DATA_POINTS];
float amplitudes[MAX_DATA_POINTS];
int dataPoints = 0;

// Sweep bookkeeping
unsigned int totalSteps = 0;
unsigned int stepIndex = 0;
unsigned int storeEvery = 1;
unsigned int currentTestFreq = 0;

// Best harmonic results
unsigned int bestHarmFreq = 0;
float bestHarmAmp = 0;

// Debounce variables for start button
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
int lastButtonState = HIGH;
int buttonState = HIGH;

// Timing variables
unsigned long lastSampleTime = 0;
unsigned long lastDisplayUpdate = 0;

// Two-pass bookkeeping
bool coarsePhase = false;
bool finePhase = false;
struct FineTarget {
  unsigned int center;
  unsigned int bestFreq;
  float bestAmp;
  bool used;
};
FineTarget fineTargets[MAX_FINE_TARGETS];
int fineTargetCount = 0;
int currentFineIndex = 0;

// Fine sweep internal state
unsigned int fineSweepFreq = 0;
unsigned int fineSweepEnd = 0;
unsigned int fineSweepStep = FINE_STEP;
unsigned long fineLastSampleTime = 0;

// Helper globals to hold last measured I
float lastIpp = 0.0f;
float lastIpk = 0.0f;

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  if (DEBUG) Serial.println("DEBUG enabled");
  pinMode(PWM_PIN, OUTPUT);
  pinMode(START_PIN, INPUT_PULLUP);

  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- Improved Band Selection Menu ---
  bool bandChosen = false;
  unsigned long pressStartTime = 0;
  bool buttonWasPressed = false;

  while (!bandChosen) {
    display.clearDisplay();
    display.setCursor(2, 2);
    display.println(F("Select Scan Range:"));
    display.setCursor(2, 20);
    display.printf("%s\n", bands[bandSelected].label);
    display.setCursor(2, 38);
    display.println(F("Press START to cycle."));
    display.setCursor(2, 50);
    display.println(F("Hold to select!"));
    display.display();

    int menuState = digitalRead(START_PIN);
    if (menuState == LOW) {
      if (!buttonWasPressed) {
        buttonWasPressed = true;
        pressStartTime = millis();
      }
    } else {
      if (buttonWasPressed) {
        unsigned long pressDuration = millis() - pressStartTime;
        if (pressDuration >= 700) {
          bandChosen = true;
        } else if (pressDuration > 20) {
          bandSelected++;
          if (bandSelected >= numBands) bandSelected = 0;
        }
        buttonWasPressed = false;
        delay(200); // debounce
      }
    }
  }

  display.clearDisplay();
  display.setCursor(2, 20);
  display.printf("Selected: %s\n", bands[bandSelected].label);
  display.setCursor(2, 38);
  display.println(F("Release button!"));
  display.display();
  delay(600);

  START_FREQ = bands[bandSelected].start;
  END_FREQ = bands[bandSelected].end;

  // Boot screen
  display.clearDisplay();
  display.setCursor(2, 2);
  display.println(F("Ultrasonic Transducer"));
  display.setCursor(2, 12);
  display.println(F("Resonance Finder"));
  display.setCursor(2, 28);
  display.printf("Range: %s\n", bands[bandSelected].label);
  display.setCursor(2, 44);
  display.println(F("Press START"));
  display.setCursor(2, 54);
  display.println(F("to sweep"));
  display.display();

  stopPWM();
}

// --- MAIN LOOP ---
void loop() {
  // Check for start button (with debounce)
  int reading = digitalRead(START_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // Button pressed (LOW)
      if (buttonState == LOW && !testRunning && !testComplete) {
        testRunning = true;
        startTwoPassTest();
      }
      // Reset test after completion
      else if (buttonState == LOW && testComplete) {
        testComplete = false;
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(2, 2);
        display.println(F("Ultrasonic Transducer"));
        display.setCursor(2, 12);
        display.println(F("Resonance Finder"));
        display.setCursor(2, 28);
        display.printf("Range: %s\n", bands[bandSelected].label);
        display.setCursor(2, 44);
        display.println(F("Press START"));
        display.setCursor(2, 54);
        display.println(F("to sweep"));
        display.display();
      }
    }
  }

  lastButtonState = reading;

  // Run sweep phase
  if (testRunning) {
    if (coarsePhase) {
      processCoarseSweep();
    } else if (finePhase) {
      processFineSweep();
    }
  }
}

// --- Scan Logic Functions ---

void startTwoPassTest() {
  display.clearDisplay();
  display.setCursor(2, 20);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(F("Scanning (coarse)..."));
  display.display();

  maxAmplitude = 0;
  resonantFreq = 0;
  resonantIpk = 0;
  resonantIpp = 0;
  bestHarmFreq = 0;
  bestHarmAmp = 0;
  dataPoints = 0;
  currentTestFreq = START_FREQ;

  totalSteps = 0;
  if (END_FREQ > START_FREQ && FREQ_STEP > 0) {
    totalSteps = (END_FREQ - START_FREQ) / FREQ_STEP + 1;
  } else {
    totalSteps = 1;
  }
  stepIndex = 0;
  storeEvery = (totalSteps + (MAX_DATA_POINTS - 1)) / MAX_DATA_POINTS;
  if (storeEvery < 1) storeEvery = 1;

  if (DEBUG) {
    Serial.print("totalSteps="); Serial.print(totalSteps);
    Serial.print(" storeEvery="); Serial.println(storeEvery);
  }

  setFrequency(currentTestFreq);

  lastSampleTime = millis();
  lastDisplayUpdate = millis();

  coarsePhase = true;
  finePhase = false;
  fineTargetCount = 0;
  currentFineIndex = 0;
}

void processCoarseSweep() {
  unsigned long currentMillis = millis();

  if (currentTestFreq > END_FREQ) {
    stopPWM();
    coarsePhase = false;
    prepareFineTargets();
    if (fineTargetCount > 0) {
      if (DEBUG) Serial.println("Starting fine phase...");
      display.clearDisplay();
      display.setCursor(2, 20);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.println(F("Refining peaks..."));
      display.display();

      currentFineIndex = 0;
      startFineTarget(currentFineIndex);
      finePhase = true;
    } else {
      computeBestHarmonicFromCoarse();
      showResults();
      testRunning = false;
      testComplete = true;
    }
    return;
  }

  if (currentMillis - lastSampleTime >= STABILIZE_DELAY) {
    float sumAmplitude = 0;
    for (int i = 0; i < SAMPLES_PER_FREQ; i++) {
      measureCurrent();
      sumAmplitude += lastIpk;
      delayMicroseconds(400);
    }
    currAmplitude = sumAmplitude / SAMPLES_PER_FREQ;

    if (currAmplitude > maxAmplitude) {
      maxAmplitude = currAmplitude;
      resonantFreq = currentTestFreq;
      resonantIpk = currAmplitude;
      resonantIpp = lastIpp;
    }

    if (dataPoints < MAX_DATA_POINTS) {
      if ((stepIndex % storeEvery == 0) || dataPoints == 0 || currentTestFreq >= END_FREQ) {
        freqs[dataPoints] = currentTestFreq;
        amplitudes[dataPoints] = currAmplitude;
        dataPoints++;
      }
    }

    if (currentMillis - lastDisplayUpdate >= 500) {
      display.fillRect(0, 34, 128, 18, SSD1306_BLACK);
      display.setCursor(2, 36);
      display.setTextColor(SSD1306_WHITE);
      display.print(F("F:"));
      display.print(currentTestFreq);
      display.print(F("Hz Ipk:"));
      display.print(currAmplitude, 3);
      display.print(F("A"));
      display.display();
      lastDisplayUpdate = currentMillis;
    }

    currentTestFreq += FREQ_STEP;
    stepIndex++;
    setFrequency(currentTestFreq);
    lastSampleTime = currentMillis;
  }
}

void measureCurrent() {
  int imax = 0;
  int imin = ADC_MAX;
  const int samples = 50;
  const int spacing_us = 100;

  for (int i = 0; i < samples; i++) {
    int iraw = analogRead(I_SENSE_PIN);
    if (iraw > imax) imax = iraw;
    if (iraw < imin) imin = iraw;
    delayMicroseconds(spacing_us);
  }

  float i_adc_vpp = ((float)(imax - imin) / ADC_MAX) * VREF;
  float i_peak = (i_adc_vpp / 2.0f) / ACS_SENSITIVITY;
  float i_pp = i_adc_vpp / ACS_SENSITIVITY;

  lastIpp = i_pp;
  lastIpk = i_peak;

  if (DEBUG) {
    Serial.print("Ipp(A): "); Serial.print(lastIpp, 6);
    Serial.print(" Ipk: "); Serial.println(lastIpk, 6);
  }
}

void prepareFineTargets() {
  fineTargetCount = 0;
  if (resonantFreq == 0 || dataPoints < 3) {
    if (DEBUG) Serial.println("No resonant or insufficient coarse data for fine targets.");
    return;
  }
  struct Peak { int idx; float amp; unsigned int f; };
  Peak peaks[MAX_DATA_POINTS];
  int peakCount = 0;
  float minPeakAmp = maxAmplitude * 0.08f;
  if (minPeakAmp < 0.01f) minPeakAmp = 0.01f;

  for (int i = 1; i < dataPoints - 1; i++) {
    if (amplitudes[i] > amplitudes[i - 1] && amplitudes[i] > amplitudes[i + 1] && amplitudes[i] >= minPeakAmp) {
      peaks[peakCount].idx = i;
      peaks[peakCount].amp = amplitudes[i];
      peaks[peakCount].f = freqs[i];
      peakCount++;
      if (peakCount >= MAX_DATA_POINTS) break;
    }
  }

  for (int i = 0; i < peakCount - 1; i++) {
    int best = i;
    for (int j = i + 1; j < peakCount; j++) {
      if (peaks[j].amp > peaks[best].amp) best = j;
    }
    if (best != i) {
      Peak tmp = peaks[i];
      peaks[i] = peaks[best];
      peaks[best] = tmp;
    }
  }

  fineTargets[0].center = resonantFreq;
  fineTargets[0].bestFreq = resonantFreq;
  fineTargets[0].bestAmp = maxAmplitude;
  fineTargets[0].used = true;
  fineTargetCount = 1;

  for (int p = 0; p < peakCount && fineTargetCount < MAX_FINE_TARGETS; p++) {
    unsigned int f = peaks[p].f;
    unsigned int window = FREQ_STEP * 4;
    if ((f > resonantFreq + window) || (f + window < resonantFreq)) {
      bool dup = false;
      for (int t = 0; t < fineTargetCount; t++) {
        if (abs((int)fineTargets[t].center - (int)f) <= (int)window) { dup = true; break; }
      }
      if (!dup) {
        fineTargets[fineTargetCount].center = f;
        fineTargets[fineTargetCount].bestFreq = f;
        fineTargets[fineTargetCount].bestAmp = peaks[p].amp;
        fineTargets[fineTargetCount].used = true;
        fineTargetCount++;
      }
    }
  }

  if (DEBUG) {
    Serial.print("Prepared fine targets: ");
    for (int t = 0; t < fineTargetCount; t++) {
      Serial.print(fineTargets[t].center); Serial.print("Hz (amp=");
      Serial.print(fineTargets[t].bestAmp); Serial.print(") ");
    }
    Serial.println();
  }
}

void startFineTarget(int index) {
  if (index < 0 || index >= fineTargetCount) return;
  unsigned int center = fineTargets[index].center;
  unsigned int startF = (center > FINE_WINDOW) ? (center - FINE_WINDOW) : START_FREQ;
  unsigned int endF = center + FINE_WINDOW;
  if (endF > END_FREQ) endF = END_FREQ;
  fineSweepFreq = startF;
  fineSweepEnd = endF;
  fineSweepStep = FINE_STEP;
  fineTargets[index].bestFreq = center;
  fineTargets[index].bestAmp = 0;
  fineLastSampleTime = millis();
  setFrequency(fineSweepFreq);

  if (DEBUG) {
    Serial.print("Fine sweep "); Serial.print(index); Serial.print(" start=");
    Serial.print(fineSweepFreq); Serial.print(" end="); Serial.println(fineSweepEnd);
  }
}

void processFineSweep() {
  unsigned long currentMillis = millis();

  if (currentFineIndex >= fineTargetCount) {
    finePhase = false;
    computeBestHarmonicFromFine();
    showResults();
    testRunning = false;
    testComplete = true;
    return;
  }

  if (currentMillis - fineLastSampleTime >= STABILIZE_DELAY) {
    float sumAmp = 0;
    for (unsigned int i = 0; i < FINE_SAMPLES_PER_FREQ; i++) {
      measureCurrent();
      sumAmp += lastIpk;
      delayMicroseconds(200);
    }
    float avgAmp = sumAmp / (float)FINE_SAMPLES_PER_FREQ;

    if (avgAmp > fineTargets[currentFineIndex].bestAmp) {
      fineTargets[currentFineIndex].bestAmp = avgAmp;
      fineTargets[currentFineIndex].bestFreq = fineSweepFreq;
      if (avgAmp > maxAmplitude) {
        maxAmplitude = avgAmp;
        resonantFreq = fineSweepFreq;
        resonantIpk = avgAmp;
        resonantIpp = lastIpp;
      }
    }

    if (fineSweepFreq + fineSweepStep > fineSweepEnd) {
      currentFineIndex++;
      if (currentFineIndex < fineTargetCount) {
        startFineTarget(currentFineIndex);
      }
    } else {
      fineSweepFreq += fineSweepStep;
      setFrequency(fineSweepFreq);
    }
    fineLastSampleTime = currentMillis;

    display.fillRect(0, 34, 128, 18, SSD1306_BLACK);
    display.setCursor(2, 36);
    display.setTextColor(SSD1306_WHITE);
    display.print(F("Ref. t:"));
    display.print(currentFineIndex + 1);
    display.print(F("/"));
    display.print(fineTargetCount);
    display.print(F(" f:"));
    display.print(fineSweepFreq);
    display.print(F("Hz"));
    display.display();
  }
}

void computeBestHarmonicFromCoarse() {
  bestHarmFreq = 0;
  bestHarmAmp = 0;
  if (resonantFreq == 0 || dataPoints < 3) {
    if (DEBUG) Serial.println("No resonant or not enough coarse data for harmonic search.");
    return;
  }
  int peakIndices[MAX_DATA_POINTS];
  int peakCount = 0;
  float minPeakAmp = maxAmplitude * 0.12f;
  if (minPeakAmp < 0.01f) minPeakAmp = 0.01f;

  for (int i = 1; i < dataPoints - 1; i++) {
    if (amplitudes[i] > amplitudes[i - 1] && amplitudes[i] > amplitudes[i + 1] && amplitudes[i] >= minPeakAmp) {
      peakIndices[peakCount++] = i;
      if (peakCount >= MAX_DATA_POINTS) break;
    }
  }

  float tol = 0.12f;
  for (int p = 0; p < peakCount; p++) {
    int idx = peakIndices[p];
    float f = (float)freqs[idx];
    float ratioUp = f / (float)resonantFreq;
    int nUp = (int)round(ratioUp);
    if (nUp >= 2) {
      float errUp = fabs(ratioUp - (float)nUp);
      if (errUp <= tol && amplitudes[idx] > bestHarmAmp) {
        bestHarmAmp = amplitudes[idx];
        bestHarmFreq = freqs[idx];
      }
    }
    float ratioDown = (float)resonantFreq / f;
    int nDown = (int)round(ratioDown);
    if (nDown >= 2) {
      float errDown = fabs(ratioDown - (float)nDown);
      if (errDown <= tol && amplitudes[idx] > bestHarmAmp) {
        bestHarmAmp = amplitudes[idx];
        bestHarmFreq = freqs[idx];
      }
    }
  }

  if (bestHarmFreq == 0) {
    float bestAmp = 0;
    unsigned int bestF = 0;
    unsigned int fundamentalWindow = FREQ_STEP * 3;
    for (int i = 0; i < dataPoints; i++) {
      unsigned int f = freqs[i];
      if ((f > resonantFreq + fundamentalWindow) || (f + fundamentalWindow < resonantFreq)) {
        if (amplitudes[i] > bestAmp) {
          bestAmp = amplitudes[i];
          bestF = f;
        }
      }
    }
    if (bestF != 0) {
      bestHarmFreq = bestF;
      bestHarmAmp = bestAmp;
    }
  }
}

void computeBestHarmonicFromFine() {
  bestHarmFreq = 0;
  bestHarmAmp = 0;
  if (resonantFreq == 0) {
    if (DEBUG) Serial.println("No resonant for fine-based harmonic selection.");
    return;
  }
  float tol = 0.08f;
  for (int t = 0; t < fineTargetCount; t++) {
    unsigned int f = fineTargets[t].bestFreq;
    float amp = fineTargets[t].bestAmp;
    if (f == 0 || amp <= 0) continue;
    if (abs((int)f - (int)resonantFreq) <= (int)FINE_STEP) continue;

    float ratioUp = (float)f / (float)resonantFreq;
    int nUp = (int)round(ratioUp);
    if (nUp >= 2) {
      float errUp = fabs(ratioUp - (float)nUp);
      if (errUp <= tol && amp > bestHarmAmp) {
        bestHarmAmp = amp;
        bestHarmFreq = f;
        continue;
      }
    }
    float ratioDown = (float)resonantFreq / (float)f;
    int nDown = (int)round(ratioDown);
    if (nDown >= 2) {
      float errDown = fabs(ratioDown - (float)nDown);
      if (errDown <= tol && amp > bestHarmAmp) {
        bestHarmAmp = amp;
        bestHarmFreq = f;
        continue;
      }
    }
    if (amp > bestHarmAmp) {
      bestHarmAmp = amp;
      bestHarmFreq = f;
    }
  }

  if (bestHarmFreq == 0) {
    if (DEBUG) Serial.println("No harmonic/subharmonic found in fine targets.");
    computeBestHarmonicFromCoarse();
  }
}

void showResults() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(2, 0);
  display.print(F("RezFq:"));
  display.print(resonantFreq);
  display.println(F("Hz"));

  display.setCursor(2, 10);
  display.print(F("HarFq:"));
  if (bestHarmFreq != 0) {
    display.print(bestHarmFreq);
    display.print(F("Hz"));
  } else {
    display.println(F("n/a"));
  }

  display.setCursor(2, 20);
  display.print(F("Ipk:"));
  display.print(resonantIpk, 4);
  display.print(F(" Ipp:"));
  display.println(resonantIpp, 4);

  plotFrequencyResponse();
  display.display();
}

void plotFrequencyResponse() {
  int x0 = 6;
  int y0 = 62;
  int w = 116;
  int h = 20;

  display.drawLine(x0, y0, x0 + w, y0, SSD1306_WHITE);
  display.drawLine(x0, y0 - h, x0, y0, SSD1306_WHITE);

  if (dataPoints < 2) return;

  float plotMaxAmp = 0;
  for (int i = 0; i < dataPoints; i++) {
    if (amplitudes[i] > plotMaxAmp) plotMaxAmp = amplitudes[i];
  }
  if (plotMaxAmp == 0) plotMaxAmp = 1;

  for (int i = 1; i < dataPoints; i++) {
    int x1 = map(freqs[i - 1], START_FREQ, END_FREQ, x0, x0 + w);
    int y1 = y0 - map((int)(amplitudes[i - 1] * 1000.0f), 0, (int)(plotMaxAmp * 1000.0f), 0, h);
    int x2 = map(freqs[i], START_FREQ, END_FREQ, x0, x0 + w);
    int y2 = y0 - map((int)(amplitudes[i] * 1000.0f), 0, (int)(plotMaxAmp * 1000.0f), 0, h);

    x1 = constrain(x1, x0, x0 + w);
    y1 = constrain(y1, y0 - h, y0);
    x2 = constrain(x2, x0, x0 + w);
    y2 = constrain(y2, y0 - h, y0);

    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }

  int resX = map(resonantFreq, START_FREQ, END_FREQ, x0, x0 + w);
  resX = constrain(resX, x0, x0 + w);
  for (int yy = y0 - h; yy <= y0; yy++) {
    display.drawPixel(resX, yy, SSD1306_INVERSE);
  }

  if (bestHarmFreq != 0) {
    int harmX = map(bestHarmFreq, START_FREQ, END_FREQ, x0, x0 + w);
    harmX = constrain(harmX, x0, x0 + w);
    for (int yy = y0 - h; yy <= y0; yy += 2) {
      display.drawPixel(harmX, yy, SSD1306_WHITE);
    }
  }

  for (int t = 0; t < fineTargetCount; t++) {
    int tx = map(fineTargets[t].center, START_FREQ, END_FREQ, x0, x0 + w);
    tx = constrain(tx, x0, x0 + w);
    display.drawLine(tx, y0 - 2, tx, y0 - 4, SSD1306_WHITE);
  }
}

// --- PWM control for RP2040 ---
void setFrequency(unsigned int freq) {
  analogWriteFreq(freq); // May need to match board's core
  analogWrite(PWM_PIN, 128);
}

void stopPWM() {
  analogWrite(PWM_PIN, 0);
}