// AVR High-voltage Serial Programmer
// Originally created by Paul Willoughby 03/20/2010
// http://www.rickety.us/2010/03/arduino-avr-high-voltage-serial-programmer/
// Inspired by Jeff Keyzer http://mightyohm.com
// Serial Programming routines from ATtiny25/45/85 datasheet

// Desired fuse configuration
#define  HFUSE  0xDF   // Defaults for ATtiny25/45/85
#define  LFUSE  0x62 
// For Attiny13 use
// #define HFUSE 0xFF
// #define LFUSE 0x6A  

#include <SoftwareSerial.h>
#define  RST     10    // Output to level shifter for !RESET from transistor to Pin 1
#define  CLKOUT  9    // Connect to Serial Clock Input (SCI) Pin 2
#define  DATAIN  0    // Connect to Serial Data Output (SDO) Pin 7
#define  INSTOUT 1    // Connect to Serial Instruction Input (SII) Pin 6
#define  DATAOUT 2    // Connect to Serial Data Input (SDI) Pin 5 
#define  VCC     3    // Connect to VCC Pin 8
#define  BUTTON  8    // Button to press to start flash
#define  RX_PIN  6    // RX Pin for softser
#define  TX_PIN  7    // TX pin for softser
#define  LED_PIN 4    // Pin for activity indication

bool led_val = false;

int inByte = 0;         // incoming serial byte Computer
int inData = 0;         // incoming serial byte AVR


SoftwareSerial mySerial(RX_PIN, TX_PIN); // RX, TX

void setup()
{
  // Set up control lines for HV parallel programming
  pinMode(VCC, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(DATAOUT, OUTPUT);
  pinMode(INSTOUT, OUTPUT);
  pinMode(CLKOUT, OUTPUT);
  pinMode(DATAIN, OUTPUT);  // configured as input when in programming mode
  pinMode(BUTTON, INPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, led_val);
  
  // Initialize output pins as needed
  digitalWrite(RST, HIGH);  // Level shifter is inverting, this shuts off 12V
  
  // start serial port at 9600 bps:
//  Serial.begin(9600);/
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
  
//  establishContact();  // send a /byte to establish contact until receiver responds 
  
}


int count = 0;

void toggle_led() {
  led_val = !led_val;
  digitalWrite(LED_PIN, led_val);
}

void loop()
{
  // if we get a valid byte, run:
  delay(10);
  if (count >= 100) {
    mySerial.println("Want to flash? y/n");
    count = 0;
  }
  count++;
  bool button_state = digitalRead(BUTTON);
  bool serial_state = (mySerial.available() > 0);
  mySerial.print("Button");
  mySerial.println(button_state);
  mySerial.print("Serial1");
  mySerial.println(serial_state);
  if (serial_state || button_state) {
    // get incoming byte:
//    inByte = Serial/.read();
    char read_char = mySerial.read();
    mySerial.println(byte(inByte));
    mySerial.println("Entering programming Mode\n");
    
    if (!button_state) {
      if (read_char != 'y') {
        mySerial.println("Wrong character pressed, try again");
        return;
      }
    }
    
    toggle_led();
    mySerial.println("Flashing");

    // Initialize pins to enter programming mode
    pinMode(DATAIN, OUTPUT);  //Temporary
    digitalWrite(DATAOUT, LOW);
    digitalWrite(INSTOUT, LOW);
    digitalWrite(DATAIN, LOW);
    digitalWrite(RST, HIGH);  // Level shifter is inverting, this shuts off 12V
    
    // Enter High-voltage Serial programming mode
    digitalWrite(VCC, HIGH);  // Apply VCC to start programming process
    delayMicroseconds(20);
    digitalWrite(RST, LOW);   //Turn on 12v
    delayMicroseconds(10);
    pinMode(DATAIN, INPUT);   //Release DATAIN
    delayMicroseconds(300);
    
    //Programming mode
    
    readFuses();
    
    //Write hfuse
    mySerial.println("Writing hfuse");
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x40, 0x4C);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, HFUSE, 0x2C);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x74);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7C);
    
    //Write lfuse
    mySerial.println("Writing lfuse\n");
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x40, 0x4C);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, LFUSE, 0x2C);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x64);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6C);

    readFuses();    
    
    mySerial.println("Exiting programming Mode\n");
    digitalWrite(CLKOUT, LOW);
    digitalWrite(VCC, LOW);
    digitalWrite(RST, HIGH);   //Turn off 12v
    toggle_led();
  }
}


void establishContact() {
//  while (Serial.available() <= 0) {
//    Serial.println("Enter a character to continue");   // send an initial string
//    delay(1000);
//  }
}

int shiftOut2(uint8_t dataPin, uint8_t dataPin1, uint8_t clockPin, uint8_t bitOrder, byte val, byte val1)
{
    int i;
    int inBits = 0;
    //Wait until DATAIN goes high
    
//    while (!digitalRead(DATAIN));
    // TODO: fix me 
    
    //Start bit
    digitalWrite(DATAOUT, LOW);
    digitalWrite(INSTOUT, LOW);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
    
    for (i = 0; i < 8; i++)  {
    
        if (bitOrder == LSBFIRST) {
            digitalWrite(dataPin, !!(val & (1 << i)));
            digitalWrite(dataPin1, !!(val1 & (1 << i)));
        }
        else {
            digitalWrite(dataPin, !!(val & (1 << (7 - i))));
            digitalWrite(dataPin1, !!(val1 & (1 << (7 - i))));
        }
        inBits <<=1;
        inBits |= digitalRead(DATAIN);
        digitalWrite(clockPin, HIGH);
        digitalWrite(clockPin, LOW);
    
    }
    
    
    //End bits
    digitalWrite(DATAOUT, LOW);
    digitalWrite(INSTOUT, LOW);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);
    
    return inBits;
}

void readFuses(){
     //Read lfuse
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x68);
    inData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6C);
    mySerial.print("lfuse reads as ");
    mySerial.println(inData, HEX);
//    
    //Read hfuse
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7A);
    inData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x7E);
    mySerial.print("hfuse reads as ");
    mySerial.println(inData, HEX);
    
    //Read efuse
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x04, 0x4C);
    shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6A);
    inData = shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, 0x00, 0x6E);
    mySerial.print("efuse reads as ");
    mySerial.println(inData, HEX);
    mySerial.println(); 
}

