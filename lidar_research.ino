/*
    author: VoidArtanis
    baudrate default 9600bps

    command indexes:##########
    XY- first bit X refers to the function type, getter or setter.
        second bits Y refers to the function name
    X - Get (1) OR Set (2)
    Y - Function Index = 0,..,n
    10 => get function 0
    ex:-
    5:10; => Getter function 1 in analog functions.

    Response syntax###########
    commandtype:commandindex=response;\r
    commandtype => specifies the category of the function, digital/analog/handshake/warning/etc
    commandindex => follow XY format given above
    response => String output from sensor or other..
    ex:-
    5:10=700;

    Request syntax############
    commandtype:command;\r
    ex:-
    5:10;

*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include "printf.h"
#include <LIDARLite.h>

//CRITICAL
//*******************************************************
#define REMOTE_DEVICE true
//*******************************************************

#define THERM_PIN_0   0  //thermal a0
#define THERM_PIN_1   1  //thermal a1
#define DIG_PIN_2     2  //digital
#define CSN_PIN       4  //radio csn
#define CE_PIN        3  //radio ce
#define LED_RAD_IN    8  //radio rx led
#define LED_RAD_OUT   10 //radio tx led
#define LSR_OUT       5 //LASER

RF24 radio(CE_PIN, CSN_PIN);
Servo myservo;  // create servo object to control a servo
Servo myservoBot;
LIDARLite myLidarLite;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int serial = 7751;               // serial
String author = "DA";            // program author
String response = "";            // string to hold the response
String radioMessage = "";
int inChar;
int commandIndexX = -1;
int commandIndexY = -1;
int commandIndexZ = -1;
int commandIndex = -1;
byte type = 0;                    // the command type from C#
int sensor_tmp;
bool therm0active = false;
bool therm1active = false;
bool laserOn = false;
bool laseractive = false;
bool readone = false;
float pos = 0.0;    // variable to store the servo position
bool servoAutoSweep = false;
unsigned long time;
unsigned long prevTime = 0;
bool lidarActive = false;

bool isWireless = false;
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
byte SendPackage[32];
byte ReceivePackage[32];

void setup() {

  // initialize serial:
  Serial.begin(9600);

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  response.reserve(200);

  //set digital outputs

  pinMode(13, OUTPUT);
  pinMode(LED_RAD_IN, OUTPUT);
  pinMode(LED_RAD_OUT, OUTPUT);
  pinMode(LSR_OUT, OUTPUT);
  //servo
  myservo.attach(9);


  //async timing
  prevTime = millis();

  //lidar
  myLidarLite.begin();

  printf_begin();
  radio.begin();
  radio.setRetries(15, 15);


  if (REMOTE_DEVICE)
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1, pipes[0]);
  }
  radio.startListening();
  radio.printDetails();

  isWireless = REMOTE_DEVICE;

  //digitalWrite(LSR_OUT, HIGH);
}

void loop() {
  time = millis();

  if (!REMOTE_DEVICE) serialEvent();
  HandleRadio();
  SensorLoop();
  LaserBlink();
  ServoSweep();

}

int inp = 0;
void serialEvent() {
  while (Serial.available()) {
    inChar = Serial.read();
    inputString += (char)inChar;
    //not splitting, builds the string in parts.
    if (inChar == ';') { //semicolon ends the line
      stringComplete = true;
    }
    else  if (inChar == '!') { //string starts with !
      stringComplete = false;
      commandIndex = -1;

      inp = 0;
    }
    else if (inp == 1)
      type = inChar - '0';
    else if (inp == 3)
      commandIndexX =  inChar - '0';
    else if (inp == 4)
    { commandIndexY = inChar - '0';
      commandIndex = (commandIndexX * 10) + commandIndexY;
    }   else if (inp == 5)
    { commandIndexZ = inChar - '0';
      commandIndex = (((commandIndexX * 10) + commandIndexY)*10) + commandIndexZ;
      Serial.println(String( inputString));
    }
    inp++;
  }


  if (stringComplete) {
    if (type == 8 ) isWireless = false;
    //if this is in wireless mode, the fixed device must transfer all commands to remote device.
    if (isWireless && type != 0)
    {
      radioPrint(inputString);
    }
    else
    {
      switch (type)
      {
        case 0: //unknown
          break;
        case 1: //command
          HandleCommand();
          break;
        case 2: //info
          break;
        case 3: //error
          break;
        case 4: //warning
          break;
        case 5: //analog
          {
            HandleAnalog();
            break;
          }
        case 6: //digital
          HandleDigital();
          break;
        case 7: //handshake
          response = "7:00;";
          PrintOut();
          break;
        case 8: //switch to serial
          isWireless = false;
          printf("######");
          printf("Device switched to serial context.");
          printf("To switch back send !8:00;");
          printf("######");
          break;
        case 9: //switch to radio transceiver
          isWireless = true;
          printf("######\n\r");
          printf("Device switched to wireless context, all messages will now be transmitted.\n\r");
          printf("To switch back send !8:00;\n\r");
          printf("######\n\r");
          break;
          break;
      }
    }
    inputString = "";
    stringComplete = false;
    type = 0;

  }
}

//####################################################
//UTILS
//####################################################
void PrintOut()
{
  if (isWireless)
  {
    radioPrint(response);
    return;
  }
  else
  {
    Serial.println(response);
  }
}

void radioPrint(String str )
{
  digitalWrite(LED_RAD_OUT, HIGH);
  str.getBytes(SendPackage, 32);
  radio.stopListening();
  unsigned long t = millis();
  printf("TX START %lu...\n\r", t);
  Serial.println(str);
  bool ack = radio.write( &SendPackage, sizeof(SendPackage) );
  digitalWrite(LED_RAD_IN, HIGH);
  if (ack)
    printf("ACK RCVD, TX SUCCESS\n\r");
  else
    printf("NACK RCVD, TX FAILED\n\r");
  digitalWrite(LED_RAD_IN, LOW);
  //delay(40);
  radio.startListening();
  printf("TX END %lu...\n\r", t);
  digitalWrite(LED_RAD_OUT, LOW);
}

//####################################################
//ANALOG
//####################################################
void HandleAnalog()
{
  switch (commandIndex)
  {
    case 10: //thermal 0
      therm0active = true;
      break;
    case 20://thermal 1
      therm0active = false;
      break;
    case 11: //thermal 0
      therm1active = true;
      break;
    case 21://thermal 1
      therm1active = false;
      break;
  }
}
void SensorLoop()
{
  if (therm0active)
  {
    response = "5:10=" + String(analogRead(THERM_PIN_0)) + ";";
    PrintOut();
  }
  if (therm0active)
  {
    response = "5:11=" + String(analogRead(THERM_PIN_1)) + ";";
    PrintOut();
  }
}
//####################################################
//DIGITAL
//####################################################
void HandleDigital()
{
  switch (commandIndex)
  {
    case 10: //laser 0
      laseractive = true;
      break;
    case 20://thermal 1
      laseractive = false;
      break;
  }
}

void LaserBlink()
{
  if (!laseractive) return;
  delay(200);
  if (laserOn)
  {
    digitalWrite(DIG_PIN_2, HIGH);
  }
  else
  {
    digitalWrite(DIG_PIN_2, LOW);
  }
  laserOn = !laserOn;
}

//####################################################
//COMMAND
//####################################################
void HandleCommand()
{
  switch (commandIndex)
  {
    case 10: //servo 0 on
      pos = 0;
      servoAutoSweep = true;
      prevTime = millis();
      break;
    case 20://servo 0 off
      LidarReadSingle();
      break;
    case 11: //LASER 0 on

      digitalWrite(LSR_OUT, HIGH);
      break;
    case 21://LASER 0 off
      digitalWrite(LSR_OUT, LOW);
      break;

  }
}
int multiplier = +1;
void ServoSweep()
{
  if (servoAutoSweep)
  {
    if (pos == 181) {
      pos = 0;
      servoAutoSweep = false;
    }

    //PrintOut();

    pos += 1;
    // in steps of 1 degree
    myservo.write(pos);
    delay(10);
    response = "1:11=" + String( myLidarLite.distance()) + ";";
    PrintOut();
    response = "1:10=" + String(pos) + ";";
    PrintOut();

    prevTime = millis();

  }
  if (readone) {

    delay(10);
    response = "1:11=" + String( myLidarLite.distance()) + ";";
    PrintOut();
    readone = false;
  }
}
void LidarReadSingle()
{
  
    response = "1:21=" + String( myLidarLite.distance()) + ";";
    PrintOut();
 
}

//####################################################
//RADIO
//####################################################

void HandleRadio()
{
  // if there is data ready
  if ( radio.available() )
  {
    digitalWrite(LED_RAD_IN, HIGH);
    // Dump the payloads until we've gotten everything
    String got_time;
    bool done = false;
    while (!done)
    {
      // Fetch the payload
      done = radio.read( &ReceivePackage, sizeof(ReceivePackage) );

      Serial.println("Got payload ..." + String((char *)ReceivePackage));

      // Delay just a little bit to let the other unit
      // make the transition to receiver
      delay(20);
    }

    //    // First, stop listening so we can talk
    //    radio.stopListening();
    //
    //    // Send the final one back.
    //    radio.write( &got_time, sizeof(unsigned long) );
    //    printf("Sent response.\n\r");
    //
    //    // Now, resume listening so we catch the next packets.
    //    radio.startListening();
    if (REMOTE_DEVICE)
    {
      int offsetError = 0;
      for (int i = 0; i < 32; i++)
      {
        if ((char)ReceivePackage[i]  == '!' )  offsetError = i;
      }
      if (ReceivePackage[offsetError + 0] == '!') {
        type = ReceivePackage[offsetError + 1] - '0';
        commandIndexX = ReceivePackage[offsetError + 3] - '0';
        commandIndexY = ReceivePackage[offsetError + 4] - '0';
    
          commandIndex = (commandIndexX * 10) + commandIndexY;
     
        switch (type)
        {
          case 0: //unknown
            break;
          case 1: //command
            HandleCommand();
            break;
          case 2: //info
             myservo.write(commandIndex*2);
            break;
          case 3: //error
            break;
          case 4: //warning
            break;
          case 5: //analog
            {
              HandleAnalog();
              break;
            }
          case 6: //digital
            HandleDigital();
            break;
          case 7: //handshake
            response = "7:00;";
            digitalWrite(13, HIGH);
            PrintOut();
            break;
          case 8: //switch to serial
            isWireless = false;
            break;
          case 9: //switch to radio transceiver
            isWireless = true;
            break;
        }
      }
    }
    else
    {
      Serial.println(String((char *)ReceivePackage));
    }
  }
  digitalWrite(LED_RAD_IN, LOW);
}
