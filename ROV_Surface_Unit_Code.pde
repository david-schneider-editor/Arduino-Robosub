//***************************************
// ROV Surface unit  v 0.1              *
// Arduino-based ROV telemetry receiver *
//                                      *
//***************************************

#include <NewSoftSerial.h>      //Need soft UART for serial display

#define NULL_PIN  5         //Software UART input pin (not used)
#define DISPLAY_PIN  6     //Software UART output pin (to serial display)

byte incomingByte;
byte headingByte;
byte depthByte;

NewSoftSerial Display_UART(NULL_PIN,DISPLAY_PIN, true); // Input not used, output to display RX pin; NOTE: inverted logic level
                                                        // The serial LCD I used (an old one) requires inverted logic. If yours does not, omit ", true" from the line above.
void setup()
{
  Display_UART.begin(9600);
  delay (500);
  Serial.begin(9600);
  delay (500);
}

void loop()
{
  while (Serial.available() > 3)       //Need at least 3 bytes for one frame of telemetry data
  {
    incomingByte = Serial.read();
    
    if (incomingByte == '\r')          //Start of data frame?
    {
      headingByte = Serial.read();
      depthByte = Serial.read(); 
      
      Display_UART.print (254,BYTE);    //Clear LCD screen NOTE: You will need to adjust this to whatever's needed on the LCD you use
      Display_UART.print (1,BYTE);

      Display_UART.print ("  H: ");
      Display_UART.print ((headingByte * 2), DEC);
      
      Display_UART.print ("  D: ");
      Display_UART.print (depthByte, DEC);
      
      delay(250);                            //Show the data on the LCD screen for long enough to see
      Serial.flush();                        //Get rid of any old data that came in while showing last set
    }
  }
}
