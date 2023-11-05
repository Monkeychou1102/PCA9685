/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
/*
// Define operating range for servos
// #0
#define Min_Range_Servo_0     600
#define Max_Range_Servo_0    2400
// #1
#define Min_Range_Servo_0     600
#define Max_Range_Servo_0    2400
// #2
#define Min_Range_Servo_0     600
#define Max_Range_Servo_0    2400
// #3
#define Min_Range_Servo_0     600
#define Max_Range_Servo_0    2400
// #4
#define Min_Range_Servo_0     600
#define Max_Range_Servo_0    2400
*/
// our servo # counter
#define servonum_Max  15     // Max. number of servo in use.
uint8_t index = 0;
uint8_t buffer_Read[4] = {0};
uint8_t servonum = 0;
uint16_t microsec = 0;
char chr = 0;
char Flag_Update = 0;

//----------------------------------------------------------------------------------------------------------------
void setup() 
{
    Serial.begin(9600);
    
    Serial.println("*************************************************************************************");
    Serial.println("< Notice >");
    Serial.println("1. Baud Rate is 9600");
    Serial.println("2. ARDUINO SERIAL MONITOR setting must be NL(newline), otherwise process will fail.");        
    Serial.println("3. Input data (Max. 4 digits) for adjusting the pulse width in us.");
    Serial.println("4. Usually the operating range for SERVO is from 500 ~ 2400 us.");
    Serial.println("*************************************************************************************");
    Serial.println(" "); 
        
    pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
//    pwm.setOscillatorFrequency(27000000);
    pwm.setOscillatorFrequency(25500000);
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

    delay(10);
}

//----------------------------------------------------------------------------------------------------------
void loop() 
{
    if(Serial.available() > 0)
    {
        Serial.println("Starting Process...");
        Serial.print("Input Data = "); 
        
// Notice the following statement...
// The setting in ARDUINO SERIAL MONITOR must be NL(newline), otherwise no '\n' will be received.
        while((chr = Serial.read()) != '\n')
        {
            if(chr >= '0' && chr <= '9')
            {
                if(index < 4)
                {
                    buffer_Read[index] = chr - 48;
                    Serial.print(buffer_Read[index]);
                }
                index++;
            }              
        }
        Serial.println(" ");
        Serial.print("Index = "); Serial.println(index); 
    }
    
//----------------------------------------------------------------------------    
// Calculating input data
    if(index > 0)
    {
        if(index <= 4)
        {
            if(index == 1)
            {
                microsec = buffer_Read[0];
            }
            else if(index == 2)
            {
                microsec = buffer_Read[0]*10 + buffer_Read[1];          
            }
            else if(index == 3)
            {
                microsec = buffer_Read[0]*100 + buffer_Read[1]*10 + buffer_Read[2];      
            }
            else if(index == 4)
            {
                microsec = buffer_Read[0]*1000 + buffer_Read[1]*100 + buffer_Read[2]*10 + buffer_Read[3];    
            }
            
            Serial.print("microsec = "); Serial.print(microsec); Serial.println(" us");

//------------------------------------------------------------------------------------------
// Update PCA9685 - Operating Range of Servo: 500 ~ 2400 us           
            Serial.println("Process to updat PCA9685...");
            for(servonum = 0; servonum < servonum_Max; servonum++)
            {
                //microsec = Serial.read();       // Operating Range of Servo: 500 ~ 2400 us
                pwm.writeMicroseconds(servonum, microsec);
                delay(50);
           }
        }
        else  Serial.println("Error...");
                
        index = 0;
        Serial.println("********************END PROCESS***********************");
        Serial.println(" ");
    }
}





