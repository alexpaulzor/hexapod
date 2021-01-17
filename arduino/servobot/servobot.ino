/*
     Servo Motor Control using Arduino and PCA9685 Driver
           by Dejan, https://howtomechatronics.com
           
     Library: https://github.com/NachtRaveVL/PCA9685-Arduino
*/

// #include <Wire.h>
// #include "PCA9685.h"

// PCA9685 driver;

// PCA9685 outputs = 12-bit = 4096 steps
// 2.5% of 20ms = 0.5ms ; 12.5% of 20ms = 2.5ms
// 2.5% of 4096 = 102 steps; 12.5% of 4096 = 512 steps
// PCA9685_ServoEval pwmServo(102, 470); // (-90deg, +90deg)

// Second Servo
// PCA9685_ServoEvaluator pwmServo2(102, 310, 505); // (0deg, 90deg, 180deg)


// void setup() {
//   Wire.begin();                 // Wire must be started first
//   Wire.setClock(400000);        // Supported baud rates are 100kHz, 400kHz, and 1000kHz
//   driver.resetDevices();        // Software resets all PCA9685 devices on Wire line

//   driver.init();         // Address pins A5-A0 set to B000000
//   driver.setPWMFrequency(50);   // Set frequency to 50Hz
// }
// void loop() {
//   // driver.setChannelPWM(0, pwmServo.pwmForAngle(-30));
//   delay(1000);
//   driver.setChannelPWM(0, pwmServo.pwmForAngle(0));
//   delay(1000);
//   // driver.setChannelPWM(0, pwmServo.pwmForAngle(30));
//   delay(1000);
// }

// /*
//      Servo Motor Control using Arduino and PCA9685 Driver
//            by Dejan, https://howtomechatronics.com
           
//      Library: https://github.com/NachtRaveVL/PCA9685-Arduino
// */

#include <Wire.h>
#include "PCA9685.h"

PCA9685 driver0;
PCA9685 driver1(B000001);

// // PCA9685 outputs = 12-bit = 4096 steps
// // 2.5% of 20ms = 0.5ms ; 12.5% of 20ms = 2.5ms
// // 2.5% of 4096 = 102 steps; 12.5% of 4096 = 512 steps
PCA9685_ServoEval pwmServo(102, 470); // (-90deg, +90deg)

// // Second Servo
// // PCA9685_ServoEvaluator pwmServo2(102, 310, 505); // (0deg, 90deg, 180deg)

#define NUM_LEGS 6
#define NUM_DRIVERS 2

#define CH_HIP_0 0
#define CH_KNEE_0 1
#define CH_ANKLE_0 2
#define CH_HIP_1 3 
#define CH_KNEE_1 4 
#define CH_ANKLE_1 5 
#define CH_HIP_2 6 
#define CH_KNEE_2 7 
#define CH_ANKLE_2 8 

#define CH_HIP_3 0
#define CH_KNEE_3 1
#define CH_ANKLE_3 2
#define CH_HIP_4 3 
#define CH_KNEE_4 4 
#define CH_ANKLE_4 5 
#define CH_HIP_5 6 
#define CH_KNEE_5 7 
#define CH_ANKLE_5 8

// #define IF_SERIAL if (false)

// const PCA9685 DRIVERS[NUM_DRIVERS] = {driver0, driver1};
const int DRV_CH_HIPS[NUM_LEGS][2] = {
    {0, CH_HIP_0}, 
    {0, CH_HIP_1},
    {0, CH_HIP_2}, 
    {1, CH_HIP_3}, 
    {1, CH_HIP_4},
    {1, CH_HIP_5}
};
const int DRV_CH_KNEES[NUM_LEGS][2] = {
    {0, CH_KNEE_0}, 
    {0, CH_KNEE_1},
    {0, CH_KNEE_2}, 
    {1, CH_KNEE_3}, 
    {1, CH_KNEE_4},
    {1, CH_KNEE_5}
};
const int DRV_CH_ANKLES[NUM_LEGS][2] = {
    {0, CH_ANKLE_0}, 
    {0, CH_ANKLE_1},
    {0, CH_ANKLE_2}, 
    {1, CH_ANKLE_3}, 
    {1, CH_ANKLE_4},
    {1, CH_ANKLE_5}
};


void set_all(int channels[NUM_LEGS][2], int angle, int channels_len = NUM_LEGS) {
    for (int i = 0; i < channels_len; i++) {
        // IF_SERIAL {
        //     Serial.print("Setting drv ");
        //     Serial.print(channels[i][0]);
        //     Serial.print(" ch ");
        //     Serial.print(channels[i][1]);
        //     Serial.print(" to angle "); 
        //     Serial.print(angle);
        //     Serial.println();
        // }
        if (channels[i][0] == 0) {
            driver0.setChannelPWM(
                channels[i][1], 
                pwmServo.pwmForAngle(angle));
        } else {
            driver1.setChannelPWM(
                channels[i][1], 
                pwmServo.pwmForAngle(angle));
        }
    }
}

void setup() {
    Wire.begin();                 // Wire must be started first
    Wire.setClock(400000);        // Supported baud rates are 100kHz, 400kHz, and 1000kHz
    // IF_SERIAL Serial.begin(9600);
    // for (int i = 0; i < NUM_DRIVERS; i++) {
        // IF_SERIAL Serial.println(
        //     sprintf("Initing driver %d inited", i));
        // PCA9685 driver = DRIVERS[i];
        driver0.resetDevices();        // Software resets all PCA9685 devices on Wire line
        driver1.resetDevices();        // Software resets all PCA9685 devices on Wire line
        driver0.init();
        driver1.init();
        driver0.setPWMFrequency(50);   // Set frequency to 50Hz
        driver1.setPWMFrequency(50);
    // }
    delay(100);
    set_all(DRV_CH_HIPS, 0, NUM_LEGS);
    set_all(DRV_CH_KNEES, 0, NUM_LEGS);
    set_all(DRV_CH_ANKLES, 0, NUM_LEGS);   
}
void loop() {
    // driver.setChannelPWM(0, pwmServo.pwmForAngle(-30));
    // delay(1000);
    // driver.setChannelPWM(0, pwmServo.pwmForAngle(0));
    // delay(1000);
    // driver.setChannelPWM(0, pwmServo.pwmForAngle(30));
    // IF_SERIAL Serial.println("loop()");
    // // IF_SERIAL Serial.println("Zeroing hips");
    // set_all(DRV_CH_HIPS, 0, NUM_LEGS);
    // // Serial.println("Zeroing knees");
    // // set_all(DRV_CH_KNEES, 0, 1);
    // // Serial.println("Zeroing ankles");
    // // set_all(DRV_CH_ANKLES, 0, 1);  
    set_all(DRV_CH_HIPS, 0, NUM_LEGS);
    set_all(DRV_CH_KNEES, 0, NUM_LEGS);
    set_all(DRV_CH_ANKLES, 0, NUM_LEGS);
    delay(1000); 
    set_all(DRV_CH_HIPS, 30, NUM_LEGS);
    set_all(DRV_CH_KNEES, -30, NUM_LEGS);
    set_all(DRV_CH_ANKLES, 30, NUM_LEGS);
    delay(1000);
    set_all(DRV_CH_HIPS, 0, NUM_LEGS);
    set_all(DRV_CH_KNEES, 0, NUM_LEGS);
    set_all(DRV_CH_ANKLES, 0, NUM_LEGS);
    delay(1000);
    set_all(DRV_CH_HIPS, -30, NUM_LEGS);
    set_all(DRV_CH_KNEES, 30, NUM_LEGS);
    set_all(DRV_CH_ANKLES, -30, NUM_LEGS);
    delay(1000);
}


// /* Sweep
//  by BARRAGAN <http://barraganstudio.com>
//  This example code is in the public domain.

//  modified 8 Nov 2013
//  by Scott Fitzgerald
//  http://www.arduino.cc/en/Tutorial/Sweep
// */

// // #include <Servo.h>

// // Servo myservo;  // create servo object to control a servo
// // twelve servo objects can be created on most boards

// // int pos = 0;    // variable to store the servo position

// // void setup() {
//   // myservo.attach(9);  // attaches the servo on pin 9 to the servo object
// // }

// // void loop() {
  
//   // for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
//   //   // in steps of 1 degree
//   //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
//   //   delay(15);    
//   //   if (pos == 90) {
//   //   	delay(1000);
//   //   }                   // waits 15ms for the servo to reach the position
//   // }
//   // for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
//   //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
//   //   delay(15);   
//   //   if (pos == 90) {
//   //   	delay(1000);
//   //   }                      // waits 15ms for the servo to reach the position
//   // }
//   /*myservo.write(0);
//   delay(1000);
//   //myservo.write(-90);
//   //delay(1000);
//   myservo.write(90);
//   delay(1000);
//   myservo.write(180);
//   delay(1000);*/
// // }
// */
