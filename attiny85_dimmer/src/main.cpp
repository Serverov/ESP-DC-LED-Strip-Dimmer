#include <Arduino.h>
/*  
	=== Attiny 85 PWM/Dimmer with 3 outputs ===

	29.03.2020 (quarantine) v1.4
  01.06.2020 (quarantine) v1.5
  02.06.2020 (quarantine) v2.1
    02.06.2020 (quarantine) v2.2
	
	* v1.4 changes:
		initial speed - 30
		loop delay 10mS

  * v2.0 changes:
		Fully Reworked Serial Command Parser
	  Added red, green and blue commands
	
	=== Pinout ===
	
	pin 2(rx), pin 3(tx) - Software Serial
	pin 0 - PWM Channel1 (red)
	pin 1 - PWM Channel2 (green)
	pin 4 - PWM Channel3 (blue)
  
	
	=== COMMANDS ===
	
	{ch1:xxx} or {blue:xxx} - Set channel1 PWM xxx = 0..255 
	{ch2:xxx} or {red:xxx} - Set channel2 PWM xxx = 0..255
	{ch3:xxx} or {green:xxx} - Set channel3 PWM xxx = 0..255
	{spd:xxx} - Set led switching speed (default 10) xxx = 0..255
  {version:xxx} - show firmware version
	
    Return:
	chx:xxx (xxx = 0..255)

    Get Current Status:
	{ch1:256}
	{ch2:256}
	{ch3:256}
	{spd:256}
*/

#include <SoftwareSerial.h>
#include <util/delay.h>

void processSerialData(void);
bool addData(char nextChar);
void updateLeds();
void processCommand(char* command, char* param);
void SetPWM (int channel, int intensity);

#define rxPin 2
#define txPin 3
SoftwareSerial SoftSerial(rxPin, txPin);

// ATtiny85 pin outputs
const int Channel1 = 0;
const int Channel2 = 1;
const int Channel3 = 4;

// target channel values
int c1 = 135;
int c2 = 135;
int c3 = 135;

// actual channel values
int ac1 = 0;
int ac2 = 0;
int ac3 = 0;

const int ChannelMaxValue = 255;

int ledChangeSpeed = 15;

const char FirmwareVersion[] = "Dimmer_attiny85_v2.2_03062020";
const char COMMAND_BEGIN_CHAR = '{';
const char COMMAND_END_CHAR = '}';
char COMMAND_DIVIDER_CHAR[] = ":";

bool cmd_begin_detected = false;

boolean ValidCommand = false;

const byte COMMAND_LENGTH = 15;

char g_buffer[COMMAND_LENGTH + 1];
char cmd_buffer[COMMAND_LENGTH + 1];

char command[COMMAND_LENGTH + 1];
char param[COMMAND_LENGTH + 1];

//char* command;
//char* param;

char receivedChars[COMMAND_LENGTH+1];
boolean newData = false;
char receivedNewData[COMMAND_LENGTH+1];

volatile uint8_t* Port[] = {&OCR0A, &OCR0B, &OCR1B};

void setup() {
  
  pinMode(Channel1, OUTPUT);
  pinMode(Channel2, OUTPUT);
  pinMode(Channel3, OUTPUT);

  digitalWrite(Channel1, LOW);
  digitalWrite(Channel2, LOW);
  digitalWrite(Channel3, LOW);
  
  /*
    Control Register A for Timer/Counter-0 (Timer/Counter-0 is configured using two registers: A and B)
    TCCR0A is 8 bits: [COM0A1:COM0A0:COM0B1:COM0B0:unused:unused:WGM01:WGM00]
    2<<COM0A0: sets bits COM0A0 and COM0A1, which (in Fast PWM mode) clears OC0A on compare-match, and sets OC0A at BOTTOM
    2<<COM0B0: sets bits COM0B0 and COM0B1, which (in Fast PWM mode) clears OC0B on compare-match, and sets OC0B at BOTTOM
    3<<WGM00: sets bits WGM00 and WGM01, which (when combined with WGM02 from TCCR0B below) enables Fast PWM mode
    */
  TCCR0A = 3<<COM0A0 | 3<<COM0B0 | 3<<WGM00;
  
   /*
    Control Register B for Timer/Counter-0 (Timer/Counter-0 is configured using two registers: A and B)
    TCCR0B is 8 bits: [FOC0A:FOC0B:unused:unused:WGM02:CS02:CS01:CS00]
    0<<WGM02: bit WGM02 remains clear, which (when combined with WGM00 and WGM01 from TCCR0A above) enables Fast PWM mode
    1<<CS00: sets bits CS01 (leaving CS01 and CS02 clear), which tells Timer/Counter-0 to not use a prescalar
    */
  TCCR0B = 0<<WGM02 | 1<<CS00;
  
   /*
    General Control Register for Timer/Counter-1 (this is for Timer/Counter-1 and is a poorly named register)
    GTCCR is 8 bits: [TSM:PWM1B:COM1B1:COM1B0:FOC1B:FOC1A:PSR1:PSR0]
    1<<PWM1B: sets bit PWM1B which enables the use of OC1B (since we disabled using OC1A in TCCR1)
    2<<COM1B0: sets bit COM1B1 and leaves COM1B0 clear, which (when in PWM mode) clears OC1B on compare-match, and sets at BOTTOM
    */
  GTCCR = 1<<PWM1B | 3<<COM1B0;
  
   /*
    Control Register for Timer/Counter-1 (Timer/Counter-1 is configured with just one register: this one)
    TCCR1 is 8 bits: [CTC1:PWM1A:COM1A1:COM1A0:CS13:CS12:CS11:CS10]
    0<<PWM1A: bit PWM1A remains clear, which prevents Timer/Counter-1 from using pin OC1A (which is shared with OC0B)
    0<<COM1A0: bits COM1A0 and COM1A1 remain clear, which also prevents Timer/Counter-1 from using pin OC1A (see PWM1A above)
    1<<CS10: sets bit CS11 which tells Timer/Counter-1  to not use a prescalar
    */
    
  TCCR1 = 0<<PWM1A | 3<<COM1A0 | 1<<CS10;

  //SetPWM(0, 0);
  //SetPWM(1, 0);
  //SetPWM(2, 0);
  
  SoftSerial.begin(9600);
}

void loop() {

// Here we process data from serial line
processSerialData();  

updateLeds();
_delay_ms(10);              // 10ms delay  

}

void processSerialData(void) {
  int data;
  bool dataReady;


  while ( SoftSerial.available() > 0 ) {
      data = SoftSerial.read();
      dataReady = addData((char)data);  
      
      if ( dataReady ) {
         strncpy(cmd_buffer, g_buffer, COMMAND_LENGTH + 1);

        char *command = strtok(cmd_buffer,  COMMAND_DIVIDER_CHAR);
        //SoftSerial.print(command);
        
        char *param = strtok(NULL,  COMMAND_DIVIDER_CHAR);
        //SoftSerial.print(param);

        processCommand(command, param);
      }
     
  }
}

// Sets channel ch1 ch2 ch3  to specified intensity 0 (off) to 255 (max)
void SetPWM (int channel, int intensity) {
  *Port[channel] = 255 - intensity;
}

// Put received character into the buffere.
// When a complete command is received return true, otherwise return false.
// The command is terminated by Enter character ("\r")
bool addData(char nextChar)
{  
  // This is position in the buffer where we put next char.
  // Static var will remember its value across function calls.
  static uint8_t currentIndex = 0;
    

    // Ignore some characters - new line, space and tabs
    if ((nextChar == '\n') || (nextChar == ' ') || (nextChar == '\t'))
        return false;

    if (nextChar == COMMAND_BEGIN_CHAR) {
        currentIndex = 0;
        cmd_begin_detected = true;
        return false;
      }

    // If we receive Enter character...
    if (nextChar == COMMAND_END_CHAR) {
        // ...terminate the string by NULL character "\0" and return true
        g_buffer[currentIndex] = '\0';
        currentIndex = 0;
        return true;
    }

    // For normal character just store it in the buffer and move
    // position to next
    if (cmd_begin_detected) {
      g_buffer[currentIndex] = nextChar;
      currentIndex++;
    }

    // Check for too many chars
    if (currentIndex >= COMMAND_LENGTH) {
      // The data too long so reset our position and return true
      // so that the data received so far can be processed - the caller should
      // see if it is valid command or not...
//      g_buffer[COMMAND_LENGTH] = '\0';
      currentIndex = 0;
      g_buffer[currentIndex] = '\0';
      cmd_begin_detected = false;
      return false;
    }

    return false;
}

void processCommand(char* command, char* param) {
	
  int intparam = 0;	
  bool getstatus = false;
  
  ValidCommand = true;
  intparam = atoi(param);
  
  if (intparam == 256) getstatus = true;
  else if (intparam > ChannelMaxValue)    intparam = ChannelMaxValue;
  else if (intparam < 0)   intparam = 0;
 
  // commands should be in the format "<COMMAND:PARAMER>"
  if (strcmp(command, "ch1") == 0) { 
    if(getstatus == false) 
	   c1 = intparam; 
	else 
	   intparam = c1;
	}
  else if (strcmp(command, "ch2") == 0) { 
	if(getstatus == false) 
		c2 = intparam; 
	else 
		intparam = c2;
	}
  else if (strcmp(command, "ch3") == 0) { 
	if(getstatus == false) 
		c3 = intparam; 
	else 
		intparam = c3;
	}

  else if (strcmp(command, "blue") == 0) { 
	if(getstatus == false) 
		c1 = intparam; 
	else 
		intparam = c1;
	}

  else if (strcmp(command, "red") == 0) { 
	if(getstatus == false) 
		c2 = intparam; 
	else 
		intparam = c2;
	}

  else if (strcmp(command, "green") == 0) { 
	if(getstatus == false) 
		c3 = intparam; 
	else 
		intparam = c3;
	}

  else if (strcmp(command, "ver") == 0) { 
    SoftSerial.print(FirmwareVersion);
    ValidCommand = false;
	}

  else if (strcmp(command, "spd") == 0) { 
	if(getstatus == false) 
		ledChangeSpeed = intparam; 
	else 
		intparam = ledChangeSpeed;
	}
  else ValidCommand = false;
  
  //Result
  if (ValidCommand == true) {
    SoftSerial.print(command);
    SoftSerial.print(":");
	SoftSerial.print(intparam);
   }
}  

void updateLeds() {
  
  // slide values towards targets, unless the distance is less then the 'slide distance' in which case, 
  // just set them to the value to prevent rubber-banding.

  if (abs(c1 - ac1) < ledChangeSpeed) {

    ac1 = c1;
    
  } else {

    if (c1 < ac1) 
      ac1 -= ledChangeSpeed; 
    else if (c1 > ac1) 
      ac1 += ledChangeSpeed;
  }

  if (abs(c2 - ac2) < ledChangeSpeed) {

    ac2 = c2;
    
  } else {

    if (c2 < ac2) 
      ac2 -= ledChangeSpeed; 
    else if (c2 > ac2) 
      ac2 += ledChangeSpeed;
  }

  if (abs(c3 - ac3) < ledChangeSpeed) {

    ac3 = c3;
    
  } else {

    if (c3 < ac3) 
      ac3 -= ledChangeSpeed; 
    else if (c3 > ac3) 
      ac3 += ledChangeSpeed;
  }

  // apply actual led values. 
  	SetPWM(0, ac1);
	SetPWM(1, ac2);
	SetPWM(2, ac3);
}