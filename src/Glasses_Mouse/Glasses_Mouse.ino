/****************************************************************************
 * Glasses Mouse
 *
 * Move a computer mouse using a microcontroller and a motion tracker (Gyro
 * + accelerometer [+magnetometer]) mounted on headgear (e.g., glasses).
 *
 * 2022-04-03 - Initial release based on code cited below.
 *
 * This code is an Arduino sketch compiled using the Arduino IDE and libraries.
 * It is being developed using:
 *   * an Adafruit QT Py SAMD21 Dev Board (#4600) MCU, and
 *   * the Adafruit MPU-6050 6-DoF Accel and Gyro Sensor (#3886)
 * connected to a host PC via a USB-C cable.
 *
 * Full credit to Jeff Ebin for the initial script from here:
 *   https://create.arduino.cc/projecthub/jeot613/glasses-mouse-control-blink-sensor-6253fc
 *
 * I've #ifdef'd the blink detector and mouse click code since that feature
 * was not desired for this effort.
 *
 * I've added:
 *   * offset, scaling, threshold, limit and timing parameters
 *   * support for the Arduino IDE Serial Plotter for realtime developer
 *     feedback
 *   * swapped the directions around such that left and down generate negative
 *     graph values.
 *
 * Where to from here?
 *   * possibly apply sensor fusion functions to use pitch and yaw as inputs.
 *     Initial attempts with G+A or just gyro yielded large swings, too much
*      drift and oscillations. No 9-axis code has been tested.
 *   * filter out involuntary movments
 *   * support newer and more accurate motion sensors.
 ****************************************************************************/

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Mouse.h>
#include <Streaming.h>

#ifndef _USING_HID
  #error "HID not supported by this board."
#endif

MPU6050 mpu;

#define BLINK_SENSOR 0

int16_t ax, ay, az, gx, gy, gz;
int     vx, vy;

const int SCALING = 100;  // lower value for more mouse movement
const int OFFSET_X = 100; // higher value for more left movement
const int OFFSET_Y = 200; // higher value for more up movement
const int MAX_VX  = 100;  // limit maximum mouse movement per read
const int MAX_VY  = 100;
const int MIN_VX  = 5;    // ignore movements below this value
const int MIN_VY  = 5;
const int READ_DELAY_ms = 100;  // delay between read/move loops in milliseconds

void setup() {
  Serial.begin(9600);
  delay(500);
  Wire.begin();
  mpu.initialize(); // sets full scale MPU6050_GYRO_FS_250 & MPU6050_ACCEL_FS_2
  while (!mpu.testConnection()) {
    Serial.println("mpu connection failed");
    delay(10e3);
  }
} // setup()

void loop() {
  // retrieve raw accelerometer and gyroscope measurements
  // angular velocity (deg/s) = g_xyz/(131 LSB/deg/s) (FS_SEL 0, +/-250 deg/s range)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // polarity changed: left and down are negative
  vx = -(gx+OFFSET_X)/SCALING;  // was: change 300 from 0 to 355
  vy =  (gz+OFFSET_Y)/SCALING;  // was: same here about "-100"  from -355 to 0
  // data for Arduino IDE Serial Plotter
  // format: <VarName>=<legendValue>:>graphValue>[,vn=lv:gv...]<eol>
/* unconstrained values:
  Serial <<  "X=" << vx << ":" << vx
         << ",Y=" << vy << ":" << vy;
*/
  // apply thresholds
  if (abs(vx) < MIN_VX) vx = 0;
  if (abs(vy) < MIN_VY) vy = 0;
  vx = (vx >= 0) ? min(vx, MAX_VX) : max(vx, -MAX_VX);
  vy = (vy >= 0) ? min(vy, MAX_VY) : max(vy, -MAX_VY);

  if (vx != 0 or vy != 0) {
    Mouse.move(vx, -vy);
  }
  // graph constrained values and sustain vertical scale (no auto-scaling)
  // Serial << ",X2=" << vx << ":" << vx
  Serial <<  "Xc=" << vx << ":" << vx
         << ",Yc=" << vy << ":" << vy
         << ",t:" << 95
         << ",b:" << -95
         << endl;
  delay(READ_DELAY_ms);

// NO blink sensor on pin A0 == NO mouse clicks
#if BLINK_SENSOR
  int numReads = 3;
  int sensorSum = 0;

  for (int k = 0; k < numReads; k++){
    sensorSum += analogRead(A0);
    delay(1);
  }

  int senseAve = sensorSum / numReads;
  delay(20);

  int current = analogRead(A0);
  int change = current - senseAve;

  Serial.println(change);
  delay(20);

  if (change < -10 || change > 10){
    digitalWrite(LED_BUILTIN, HIGH);
    Mouse.press(MOUSE_LEFT);
    delay(200);
  }

  if (change  > -3 && change < 3) {
    digitalWrite(LED_BUILTIN, LOW);
    Mouse.release(MOUSE_LEFT);
  }
#endif

} // loop()

