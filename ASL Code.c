#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// === Touch sensor pins ===
const int SENSOR_PINS[4] = {4, 5, 6, 7}; // Using only T1, T2, T3 for detection
const int touchNumReadings = 3; // smoothing length

int touchReadings[4][touchNumReadings];
int touchIndex[4] = {0, 0, 0, 0};

// === Flex sensor pins ===
const int flexPin1 = A0;
const int flexPin2 = A1;
const int flexPin3 = A2; // Ignored in detection logic but printed
const int flexPin4 = A3;

// === Flex smoothing parameters ===
const int numReadings = 10;
int readings1[numReadings], readings2[numReadings], readings3[numReadings], readings4[numReadings];
int readIndex1=0, readIndex2=0, readIndex3=0, readIndex4=0;
long total1=0, total2=0, total3=0, total4=0;
int average1, average2, average3, average4;

// Flex calibration min/max
const int flex1Min = 900, flex1Max = 1023;
const int flex2Min = 850, flex2Max = 960;
const int flex3Min = 765, flex3Max = 783;
const int flex4Min = 890, flex4Max = 965;

Adafruit_MPU6050 mpu;

// -------- LETTER DETECTION FUNCTION --------
// f1b, f2b, f4b = 1 bent, 0 straight
// t1,t2,t3 = smoothed touch states
// g = gyro, a = accelerometer
char detectLetter(int f1b, int f2b, int f4b,
                  int t1, int t2, int t3, sensors_event_t &g, sensors_event_t &a) {

  // ----- STATIC LETTERS -----
  if (f1b && f2b && f4b && t1 && t2 && t3) {
    return 'A';
}        // All bent, no f1b && f2b && f4b && !t1 && !t2 && !t3
  if (!f1b && !f2b && !f4b && t1 && !t2 && !t3) return 'B';        // All straight, thumb touches index+middle
  //if (f1b && f2b && f4b && !t1 && t2 && t3) return 'C';            // Fist thumb under, middle+ring touch
  if (!f1b && f2b && f4b && t1 && !t2 && !t3 && t3) return 'D';         // Index straight, rest bent, index touch
  if (f2b && f4b && t1) return 'E';           // Hooked index, thumb touch
  if (f1b && f2b && f4b && !t1 && t2 && t3 && (a.acceleration.z < 4)) {
    return 'M';
}        // Curved outer fingers, middle straight, touch
  if (!f1b && f2b && f4b && !t1 && !t2 && (a.acceleration.z > 2)) return 'P';
          // Fist bent, thumb touch index
  if (f1b && f2b && !f4b && t1) return 'I';
  if (!f1b && !f2b && f4b && t1) return 'U';        // Two fingers straight, no touch  
  if (f2b && f4b && !t2 && t3 && !t1) return 'T';      
  /*if (!f1b && f2b && !f4b && !t1 && t2 && t3) return 'F';         // OK sign, middle+ring touch
  //if (!f1b && f2b && !f4b && (a.acceleration.y > 6)) return 'G';   // Sideways finger-gun (accel Y)
  if (!f1b && !f2b && f4b && t1 && t2 && !t3) return 'H';          // Two fingers up + thumb touches
 
  //if (!f1b && f2b && f4b && !t1 && t2 && !t3) return 'K';          // K sign thumb touches middle
  //if (!f1b && !f2b && f4b && !t1 && !t2 && !t3) return 'L';        // L shape
  
  //if (f1b && f2b && !f4b && !t1 && t2 && t3) return 'N';           // Two bent fingers over thumb (middle+ring touch)
  //if (f1b && f2b && f4b && !t1 && !t2 && !t3) return 'O';             // All bent, all touchf1b && f2b && f4b && t1 && t2 && t3
  // P: Index straight, other fingers bent, thumb touches middle, hand facing down
  

  if (f1b && f2b && !f4b && (a.acceleration.z < -7)) return 'Q';    // Bent index+middle, hand down
  //if (!f1b && !f2b && f4b && !t1 && !t2 && !t3) return 'R';        // Crossed fingers approx.
  
  
  //if (!f1b && !f2b && f4b && !t1 && !t2 && !t3) return 'V';        // Similar to U (fingers apart)
  //if (!f1b && !f2b && !f4b && !t1 && !t2 && !t3) return 'W';       // All straight, no touch
  
  if (!f1b && f2b && !f4b && (a.acceleration.z < -6)) return 'Y';   // Y shape with hand down orientation

  // ----- MOTION LETTERS -----
  //if (!f1b && f2b && f4b && fabs(g.gyro.y) > 1.0) return 'J';      // J motion
  if (!f1b && f2b && !f4b && fabs(g.gyro.x) > 1.0) return 'Z';     // Z motion*/

  return '-'; // no match
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
    for (int j=0;j<touchNumReadings;j++) touchReadings[i][j] = 0;
  }

  average1 = analogRead(flexPin1);
  for (int i=0; i<numReadings; i++) { readings1[i]=average1; total1+=average1; }
  average2 = analogRead(flexPin2);
  for (int i=0; i<numReadings; i++) { readings2[i]=average2; total2+=average2; }
  average3 = analogRead(flexPin3);
  for (int i=0; i<numReadings; i++) { readings3[i]=average3; total3+=average3; }
  average4 = analogRead(flexPin4);
  for (int i=0; i<numReadings; i++) { readings4[i]=average4; total4+=average4; }

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
  // Smooth flex readings
  int raw1 = analogRead(flexPin1);
  total1 -= readings1[readIndex1];
  readings1[readIndex1] = raw1;
  total1 += raw1;
  readIndex1 = (readIndex1 + 1) % numReadings;
  average1 = total1 / numReadings;

  int raw2 = analogRead(flexPin2);
  total2 -= readings2[readIndex2];
  readings2[readIndex2] = raw2;
  total2 += raw2;
  readIndex2 = (readIndex2 + 1) % numReadings;
  average2 = total2 / numReadings;

  int raw3 = analogRead(flexPin3);
  total3 -= readings3[readIndex3];
  readings3[readIndex3] = raw3;
  total3 += raw3;
  readIndex3 = (readIndex3 + 1) % numReadings;
  average3 = total3 / numReadings;

  int raw4 = analogRead(flexPin4);
  total4 -= readings4[readIndex4];
  readings4[readIndex4] = raw4;
  total4 += raw4;
  readIndex4 = (readIndex4 + 1) % numReadings;
  average4 = total4 / numReadings;

  // Convert to degrees
  int flex1Deg = constrain(map(average1, flex1Min, flex1Max, 0, 90), 0, 90);
  int flex2Deg = constrain(map(average2, flex2Min, flex2Max, 0, 90), 0, 90);
  int flex3Deg = constrain(map(average3, flex3Min, flex3Max, 0, 90), 0, 90);
  int flex4Deg = constrain(map(average4, flex4Min, flex4Max, 0, 90), 0, 90);

  // Bent/straight logic (>60° bent = 1)
  int bentF1 = (flex1Deg > 60) ? 1 : 0;
  int bentF2 = (flex2Deg > 77) ? 1 : 0;
  int bentF4 = (flex4Deg > 70) ? 1 : 0;

  // Smooth touch readings
  int rawTouch[4], smoothTouch[4];
  for (int i = 0; i < 4; i++) {
    rawTouch[i] = digitalRead(SENSOR_PINS[i]) ? 1 : 0;
    touchReadings[i][touchIndex[i]] = rawTouch[i];
    touchIndex[i] = (touchIndex[i] + 1) % touchNumReadings;
    int ones = 0;
    for (int j=0; j<touchNumReadings; j++) if (touchReadings[i][j]) ones++;
    smoothTouch[i] = (ones > touchNumReadings / 2) ? 1 : 0;
  }

  // Get MPU readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Detect letter using bent/straight flex, smoothed touch, and gyro/accel
  char letter = detectLetter(bentF1, bentF2, bentF4,
                             smoothTouch[0], smoothTouch[1], smoothTouch[2], g, a);

  // ===== SERIAL OUTPUT =====
  Serial.print(letter);
  /*Serial.print(" | Touch Raw: ");
  for (int i=0; i<4; i++) {
    Serial.print(rawTouch[i]);
    if (i < 3) Serial.print(", ");
  }*/
  /*Serial.print(" | Touch Smoothed: ");
  for (int i=0; i<3; i++) {
    Serial.print(smoothTouch[i]);
    if (i < 3) Serial.print(", ");
  }

  Serial.print(" | Flex1 raw: "); Serial.print(average1); Serial.print(" (deg): "); Serial.print(flex1Deg);
  Serial.print(" | Flex2 raw: "); Serial.print(average2); Serial.print(" (deg): "); Serial.print(flex2Deg);
  //Serial.print(" | Flex3 raw: "); Serial.print(average3); Serial.print(" (deg): "); Serial.print(flex3Deg);
  Serial.print(" | Flex4 raw: "); Serial.print(average4); Serial.print(" (deg): "); Serial.print(flex4Deg);

  Serial.print(" | Acc: ");
  Serial.print(a.acceleration.x, 2); Serial.print(", ");
  Serial.print(a.acceleration.y, 2); Serial.print(", ");
  Serial.print(a.acceleration.z, 2);

  Serial.print(" | Gyro: ");
  Serial.print(g.gyro.x, 2); Serial.print(", ");
  Serial.print(g.gyro.y, 2); Serial.print(", ");
  Serial.print(g.gyro.z, 2);
 */
  Serial.println();
  delay(100);
}
