# Glasses_Mouse
Move computer mouse using microcontroller and motion sensor board.

This code is an Arduino sketch compiled using the Arduino IDE and libraries.
It is being developed using:
  * an Adafruit QT Py SAMD21 Dev Board (#4600) MCU, and
  * the Adafruit MPU-6050 6-DoF Accel and Gyro Sensor (#3886)
  * connected to a host PC via a USB-C cable.

Full credit to Jeff Ebin for the initial script from here:
 * https://create.arduino.cc/projecthub/jeot613/glasses-mouse-control-blink-sensor-6253fc

I've added:
  * offset, scaling, threshold, limit and timing parameters
  * support for the Arduino IDE Serial Plotter for realtime developer feedback
  * swapped the directions around such that left and down generate negative graph values.
