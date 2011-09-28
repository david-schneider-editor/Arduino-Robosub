/*
 * ROV in-sub unit version 0.1
 * June 2011
 */

// This is the code that runs on the Arduino Pro Mini housed within the ROV itself. It serves two functions:
// 1. It received the PPM signal sent down from the surface unit, mixes two of the channels and sends the appropriate
//    PWM signals to the motor controllers ("electronic speed controllers" or ESCs) and camera-tilt servo.
//
// 2. It interfaces with the digital compass and takes voltage measurements of the output of the pressure sensor. These
//    sensor measurements are then put into a three-byte package (a depth byte, a heading byte, and a carriage return),
//    which is sent to the surface for display. That allows 1-foot depth resolution, and 2-degree heading resolution,
//    which is more than sufficient for this application.
//
// The low-level code used to interface the HMC6352 digital compass is courtesy of Lars Schumann. See http://make.larsi.org/electronics/HMC6352/
//
#include <Servo.h>
#include <Wire.h>

#define channumber 4            //Set to # of channels on RC transmitter being used
                                //This code assumes channel 0 is "elevator" (fwd and rev on right stick)
                                //                  channel 1 is "ailerons" (right and left on right stick)
                                //                  channel 2 is "throttle" (right and left on left stick)
                                //                  channel 3 is "rudder" (fwd and rev on left stick)
                                //You will have to modify "value[x]" accordingly in the code if your radio has different channel assignments
                                //On my setup, channel 0 and 1 are used to control the left and right thrusters (by mixing the channels),
                                //I used channel 2 (throttle) for the middle (up/down) thruster and channel 3 (rudder) to control the camera tilt.

#define neutralPW  1100         //Neutral pulse width from radio (microseconds); measure the PPM signal from your RC radio on an oscilloscope and adjust value to match
#define minPW 600               //Minimum pulse width from radio; measure PPM from your RC radio and adjust value to match
#define maxPW 1600              //Maximum pulse width from radio; measure PPM from your RC radio and adjust value to match
#define PPM_PIN 12              //Send the PPM signal to this pin on the Arduino
#define RIGHT_THRUSTER_PIN 6    //PWM output to ESC controlling right thruster
#define LEFT_THRUSTER_PIN 8     //PWM output to ESC controlling left thruster
#define CENTER_THRUSTER_PIN 7   //PWM output to ESC controlling center thruster
#define CAMERA_SERVO_PIN 13     //PWM output to control camera-tilt servo

long value[channumber];         //Channel pulse widths from PPM signal (channels 0 to 3)
long old_camera_servo_value;    //Needed to prevent camera servo from jiggling
long camera_deadband = 25;
int right_thruster_pw;          //Output to right-thruster ESC
int left_thruster_pw;           //Output to left-thruster ESC
int center_thruster_pw;         //Output to center-thruster ESC
int camera_servo_pw;            //Output to camera servo

Servo right_thruster;           //Servo object to control the right thruster
Servo left_thruster;            //Servo object to control the left thruster
Servo center_thruster;          //Servo object to control the center thruster
Servo camera_servo;             //Servo object to control the camera-tilt servo

int HMC6352Address = 0x42;      //Digital compass
int slaveAddress;               //Calculated in Setup

byte headingData[2];
int i;
int j;
int x;
int headingValue;
int temp;
int depth;
int pressureZero;                          //Pressure sensor output at surface
int counter = 0;

void setup()
{
// Serial.begin(9600);                     //Needed only for testing
// Serial.println ("Initializing");
  
  //Stuff for servo/esc control

  pinMode(PPM_PIN, INPUT);                 //Pin for PPM signal needs to be an input
  pinMode(RIGHT_THRUSTER_PIN, INPUT);      //Pin for PPM signal needs to be an output
  pinMode(LEFT_THRUSTER_PIN, INPUT);       //Pin for PPM signal needs to be an output
  pinMode(CENTER_THRUSTER_PIN, INPUT);     //Pin for PPM signal needs to be an output
  pinMode(CAMERA_SERVO_PIN, INPUT);        //Pin for PPM signal needs to be an output
  
  right_thruster.attach(RIGHT_THRUSTER_PIN);   //PWM signal for right thruster on Pin 6
  left_thruster.attach(LEFT_THRUSTER_PIN);    //PWM signal for right thruster on Pin 7
  center_thruster.attach(CENTER_THRUSTER_PIN);  //PWM signal for right thruster on Pin 8
  camera_servo.attach(CAMERA_SERVO_PIN);     //PWM signal for camera-tilt servo on Pin 9
  
  // Stuff for HMC6352 digital compass
  // Here we shift the device's documented slave address (0x42) 1 bit right

  // This compensates for how the TWI library only wants the
  // 7 most significant bits (with the high bit padded with 0)
  
  slaveAddress = HMC6352Address >> 1;  
  
  // This results in 0x21 as the address to pass to TWI

  Wire.begin();
  
  pressureZero = analogRead(0);                   //Pressure sensor output at surface
 
}

void loop()
{
  //Zero the value[x] array
  for(x=0; x<=channumber-1; x++)            
  {
    value[x] = 0; 
  }
  
  for (j=0; j<5; j++)                               //Read PPM frames 5 times and average
  {  
    while(pulseIn(PPM_PIN, LOW) < 5000){}           //Wait for the beginning of the frame
  
    for(x=0; x<=channumber-1; x++)                  //Loop to store all the channel position
    {
      value[x] = value[x] + pulseIn(PPM_PIN, LOW);  //Accumulate values for averaging
    }
  }
  
  for(x=0; x<=channumber-1; x++)                    //Divide to get average value
  {
    value[x] = value[x] / 5;
  }
  
  if (abs(value[2] - old_camera_servo_value) < camera_deadband)  //To prevent camera from jiggling
  {                                                              // Don't move unless target is outside deadband
    value[2] = old_camera_servo_value;
  }
  else
  {
    old_camera_servo_value = value[2];
  }

  Mix ();                                                 //Mix channels 0 and 1 so as to control left and right thrusters properly

  right_thruster.writeMicroseconds(value[0] + 400);       //Output for ESC
  left_thruster.writeMicroseconds(value[1] + 400);        //Output for ESC
  center_thruster.writeMicroseconds(value[3] + 400);      //Output for ESC controlling center thruster (use rudder stick to control)
  camera_servo.writeMicroseconds(value[2] + 400);         //Output for camera-tilt servo (use throttle stick to control)
  
  GetHeading ();                                          //Determine heading
  GetDepth ();                                            //Determine depth
  Send_Data_Topside();                                    //Send compass heading and pressure depth data to the surface (each 1/4 second)
} 

//Subroutines
//
//This routine mixes the aileron and elevator channels so that pushing the right stick forward powers both right and
//left thrusters, making the ROV go forward. Pushing the right stick to one side, makes the ROV turn.
//Note that you have to check that the mixed signals don't exceed normal bounds of PWM width when the stick is pushed
//both forward or revese and to one side.
void Mix ()
{
  int right_out;
  int left_out;
  
  right_out = (value[0] + (value[1] - neutralPW));

  if (right_out > maxPW)
  {
    right_out = maxPW;
  }

  if (right_out < minPW)
  {
    right_out = minPW;
  }

  left_out = (value[0] - (value[1] - neutralPW));
  
  if (left_out > maxPW)
  {
    left_out = maxPW;
  }

  if (left_out < minPW)
  {
    left_out = minPW;
  }

  value[0] = right_out;
  value[1] = left_out;
}

void GetHeading ()
{
  // Send an "A" command to the HMC6352
  Wire.beginTransmission(slaveAddress);
  Wire.send("A");              // The "Get Data" command
  Wire.endTransmission();
  delay(10);
  // The HMC6352 needs at least a 70us (microsecond) delay
  // after this command.  Using 10ms just makes it safe
  // Read the 2 heading bytes, MSB first
  // The resulting 16bit word is the compass heading in 10th's of a degree
  // For example: a heading of 1345 would be 134.5 degrees

  Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first

  i = 0;

  while(Wire.available() && i < 2)
  {
    headingData[i] = Wire.receive();
    i++;
  }
  headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together
}

void GetDepth()
{
  depth = analogRead(0) - pressureZero;                //Read pressure sensor and substract zero value
  depth = (int)(depth * 0.353);                        //Convert to feet of depth (from linear regression on sensor)


}

 
void Send_Data_Topside ()
{
  byte heading_byte;
  byte depth_byte;
  
  heading_byte = lowByte(headingValue/20);              //Maps 0 to 360 degrees to values in the range 0 to 180 (so it can fit in one byte)
  
  if (depth < 0)
  {
    depth_byte = 0;
  }
  else
  {
    depth_byte =  lowByte(depth);
  }

  Serial.print (heading_byte);
  Serial.print (depth_byte);
  Serial.print ('\r');                       //Print CR (without any LF)
}





