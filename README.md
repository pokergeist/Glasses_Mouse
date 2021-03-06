# Glasses Mouse

__This project is Under Development__ while I work with a user to test different methodologies, e.g., mapping Vxy or pitch & yaw (via sensor fusion) to mouse movement (absolute, damped or progressive). This repo will be renamed to the more generic term "Head Mouse".

### Move your computer mouse using microcontroller and motion sensor board.

This code is an Arduino sketch compiled using the Arduino IDE (v1.8.13) and libraries.
It is being developed using:
  * an Adafruit QT Py SAMD21 Dev Board (#4600) MCU
  * the Adafruit MPU-6050 6-DoF Accel and Gyro Sensor (#3886) and the newer MPU-9250 board (eBay)
  * connected to a host PC via a USB-C cable.

I could likely port it to the Adafruit LSM6DSOX + LIS3MDL - Precision 9 DoF IMU (#4517) which I have on-hand.

Full credit to Jeff Ebin for the initial script from here:
 * https://create.arduino.cc/projecthub/jeot613/glasses-mouse-control-blink-sensor-6253fc

I've added:
  * offset, scaling, threshold, limit and timing parameters
  * support for the Arduino IDE Serial Plotter for realtime developer feedback
  * swapped the directions around such that left and down generate negative graph values
  * an I2C ping to test connectivity
  * a revised Device ID test that issues a warning rather than lock up.
