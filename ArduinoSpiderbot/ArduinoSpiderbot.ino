/*
 * Author:      Farris Matar
 * Date:        April 11, 2021
 * Project:     Arduino Spiderbot
 */

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// OLED display TWI address
#define OLED_ADDR   0x3C
Adafruit_SSD1306 display(-1);

// Defining servos
Servo ShoulderHLeg1;
Servo ShoulderVLeg1;
Servo kneeLeg1;
Servo leg1[3] = {ShoulderHLeg1, ShoulderVLeg1, kneeLeg1};

Servo ShoulderHLeg2;
Servo ShoulderVLeg2;
Servo kneeLeg2;
Servo leg2[3] = {ShoulderHLeg2, ShoulderVLeg2, kneeLeg2};

Servo ShoulderHLeg3;
Servo ShoulderVLeg3;
Servo kneeLeg3;
Servo leg3[3] = {ShoulderHLeg3, ShoulderVLeg3, kneeLeg3};

Servo ShoulderHLeg4;
Servo ShoulderVLeg4;
Servo kneeLeg4;
Servo leg4[3] = {ShoulderHLeg4, ShoulderVLeg4, kneeLeg4};

Servo legs[4][3] = {leg1, leg2, leg3, leg4};
Servo regLegs[2][3] = {leg1, leg3};
Servo invLegs[2][3] = {leg2, leg4};
int servoPins[4][3] = { {2, 4, 7}, {8, 9, 10}, {11, 12, 13}, {14, 15, 16} };

bool isFirstStep = true;
bool lastStepRight = false;
int speedMult = 2;

char bluetoothVal;
int redLED = 0;
int greenLED = 0;
int blueLED = 0;

// Constants for servo angles for regular legs
const int STARTING_SHLDR = 90;
const int STARTING_LEG_REG = 0;

const int STANDING_SHLDR_REG = 135;
const int STANDING_SHLDR_V_REG = 105;
const int STANDING_LEG_REG = 100;

const int EXTENDED_SHLDR_REG = 156;
const int COMPRESSED_SHLDR_REG = 79;

const int EXTENDED_REG_LEG = 115;
const int COMPRESSED_REG_LEG = 65;

const int UNRAISED_REG_LEG = 100;
const int RAISED_REG_LEG = 120;

// Constants for servo angles for inverted legs
const int STARTING_LEG_INV = 180;

const int STANDING_SHLDR_INV = 45;
const int STANDING_SHLDR_V_INV = 75;
const int STANDING_LEG_INV = 80;

const int EXTENDED_SHLDR_INV = 24;
const int COMPRESSED_SHLDR_INV = 101;

const int EXTENDED_INV_LEG = 65;
const int COMPRESSED_INV_LEG = 115;

const int UNRAISED_INV_LEG = 80;
const int RAISED_INV_LEG = 60;

void setup() {
  // Initializing OLED
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.display();
  
  delay(5000);

  // Bluetooth initialization
  Serial1.begin(9600);
  delay(25);

  // LED initialization
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  
  // Setting up servos
  for (int i = 0; i < 4; i++)
  {
    legs[i][0].write(STARTING_SHLDR);
    delay(100);
    legs[i][1].write(STARTING_SHLDR);
    delay(100);
  }
  for (int i = 0; i < 2; i++)
  {
    regLegs[i][2].write(STARTING_LEG_REG);
    delay(100);
    invLegs[i][2].write(STARTING_LEG_INV);
    delay(100);
  }
  
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      legs[i][j].attach(servoPins[i][j]);
      delay(100);
    }
  }
  delay(3000);

  stand();
  
  delay(1000);

  happy();
}

void loop() 
{
  // Getting Bluetooth command
  if (Serial1.available()) bluetoothVal = Serial1.read();
  delay(100);

  // Determining next action based on Bluetooth message
  if (bluetoothVal == 'r')
  {
    redLED = Serial1.parseInt();
  }
  else if (bluetoothVal == 'g')
  {
    greenLED = Serial1.parseInt();
  }
  else if (bluetoothVal == 'b')
  {
    blueLED = Serial1.parseInt();
  }
  else if (bluetoothVal == 'f')
  {
    cierra();
    delay(100);
    happy();
    
    if (isFirstStep)
    {
      firstStep();
      isFirstStep = false;
      lastStepRight = false;
    }
    if (!lastStepRight) rightStepForward();
    else leftStepForward();
  }
  else if (bluetoothVal == 'm')
  {
    cierra();
    delay(100);
    enfado();
    
    turnRight();
  }
  else if (bluetoothVal == 'l')
  {
    cierra();
    delay(100);
    enfado1();
    
    turnLeft();
  }
  else if (bluetoothVal == 'p')
  {
    cierra();
    delay(100);
    triste();
    
    if (isFirstStep)
    {
      firstStep();
      isFirstStep = false;
      lastStepRight = false;
    }
    if (!lastStepRight) rightStepBack();
    else leftStepBack();
  }
  else if (bluetoothVal == 't')
  {
    speedMult = 4;
  }
  else if (bluetoothVal == 'o')
  {
    speedMult = 1;
  }
  
  delay(10);
  analogWrite(3,redLED);
  delay(10);
  analogWrite(5,greenLED);
  delay(10);
  analogWrite(6,blueLED);
  delay(10);

  bluetoothVal = "";
  delay(20);
}

// Getting the robot to stand after setting up
void stand()
{
  // Raising the shoulders up
  for (int i = 0; i < 2; i++)
  {
    regLegs[i][1].write(STANDING_SHLDR_REG);
    delay(250);
  }
  for (int i = 0; i < 2; i++)
  {
    invLegs[i][1].write(STANDING_SHLDR_INV);
    delay(250);
  }

  // Turning the shoulders to 45 degrees
  for (int i = 0; i < 2; i++)
  {
    regLegs[i][0].write(STANDING_SHLDR_REG);
    delay(250);
  }
  for (int i = 0; i < 2; i++)
  {
    invLegs[i][0].write(STANDING_SHLDR_INV);
    delay(250);
  }

  // Bringing the legs down
  for (int i = 0; i < 2; i++)
  {
    regLegs[i][2].write(STANDING_LEG_REG);
    delay(250);
  }
  for (int i = 0; i < 2; i++)
  {
    invLegs[i][2].write(STANDING_LEG_INV);
    delay(250);
  }

  // Bringing the shoulders down again
  for (int i = 0; i < 2; i++)
  {
    if (i == 0) regLegs[i][1].write(STANDING_SHLDR_V_REG);
    else regLegs[i][1].write(STANDING_SHLDR_V_REG);
    delay(15/speedMult);
  }
  for (int i = 0; i < 2; i++)
  {
    invLegs[i][1].write(STANDING_SHLDR_V_INV);
    delay(15/speedMult);
  }
}

/*-------------- FORWARD & BACKWARD MOVEMENT ---------------*/

// Makes the robot take a step forward with the right leg
void rightStepForward() 
{
  extendRegLeg(leg3);

  // Turning the shoulders to drag the robot forward
  for (int i = 3; i <= 55; i+= 3)
  {
    leg4[0].write(STANDING_SHLDR_INV+i);
    delay(10/speedMult);
    
    leg2[0].write(COMPRESSED_SHLDR_INV-i);
    delay(10/speedMult);
    
    if (i < 21) leg3[0].write(EXTENDED_SHLDR_REG-i);
    else leg3[0].write(STANDING_SHLDR_REG);
    delay(10/speedMult);
    
    if (i < 21) leg1[0].write(STANDING_SHLDR_REG+i);
    else leg1[0].write(EXTENDED_SHLDR_REG);
    
    delay(10/speedMult);
  }

  // Adjusting the legs to be in the proper position
  // Vertical shoulder
  for (int i = 1; i <= 5; i++)
  {
    leg1[1].write(STANDING_SHLDR_V_REG-i);
    delay(15/speedMult);
    
    leg2[1].write(UNRAISED_INV_LEG-i);
    delay(15/speedMult);
    
    leg3[1].write(UNRAISED_REG_LEG+i);
    delay(15/speedMult);
    
    leg4[1].write(STANDING_SHLDR_V_INV+i);
    delay(15/speedMult);
  }
  // Leg
  for (int i = 3; i <= 35; i+= 3)
  {
    leg1[2].write(STANDING_LEG_REG-i);
    delay(15/speedMult);
    
    if (i < 15) leg2[2].write(EXTENDED_INV_LEG+i);
    else leg2[2].write(STANDING_LEG_INV);
    delay(15/speedMult);
    
    leg3[2].write(COMPRESSED_REG_LEG+i);
    delay(15/speedMult);
    
    if (i < 15) leg4[2].write(STANDING_LEG_INV-i);
    else leg4[2].write(EXTENDED_INV_LEG);
    delay(15/speedMult);
  }

  // Preparing leg 1 for the next step
  // Raising the leg
  for (int i = UNRAISED_REG_LEG; i <= RAISED_REG_LEG; i+= 2)
  {
    leg1[1].write(i);
    delay(15/speedMult);
  }
  // Getting into position
  for (int i = EXTENDED_SHLDR_REG; i >= COMPRESSED_SHLDR_REG; i-= 3)
  {
    leg1[0].write(i);
    delay(15/speedMult);
  }
  for (int i = COMPRESSED_REG_LEG; i <= EXTENDED_REG_LEG; i+= 3)
  {
    leg1[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_REG_LEG; i >= UNRAISED_REG_LEG; i-= 2)
  {
    leg1[1].write(i);
    delay(15/speedMult);
  }

  lastStepRight = true;
}

// Makes the robot take a step back with the left leg
void leftStepBack()
{
  // Undoing preparation of leg 1
  // Raising the leg
  for (int i = UNRAISED_REG_LEG; i <= RAISED_REG_LEG; i+= 2)
  {
    leg1[1].write(i);
    delay(15/speedMult);
  }
  // Getting into position
  for (int i = COMPRESSED_SHLDR_REG; i <= EXTENDED_SHLDR_REG; i+= 3)
  {
    leg1[0].write(i);
    delay(15/speedMult);
  }
  for (int i = EXTENDED_REG_LEG; i >= COMPRESSED_REG_LEG; i-= 3)
  {
    leg1[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_REG_LEG; i >= UNRAISED_REG_LEG; i-= 2)
  {
    leg1[1].write(i);
    delay(15/speedMult);
  }

  // Turning the shoulders to drag the robot backwards
  for (int i = 3; i <= 55; i+= 3)
  {
    leg4[0].write(COMPRESSED_SHLDR_INV-i);
    delay(10/speedMult);
    
    leg2[0].write(STANDING_SHLDR_INV+i);
    delay(10/speedMult);
    
    if (i < 21) leg3[0].write(STANDING_SHLDR_REG+i);
    else leg3[0].write(EXTENDED_SHLDR_REG);
    delay(10/speedMult);
    
    if (i < 21) leg1[0].write(EXTENDED_SHLDR_REG-i);
    else leg1[0].write(STANDING_SHLDR_REG);
    delay(10/speedMult);
  }

  // Adjusting the legs to be in the proper position
  // Vertical shoulder
  for (int i = 1; i <= 5; i++)
  {
    leg1[1].write(UNRAISED_REG_LEG+i);
    delay(15/speedMult);
    
    leg2[1].write(STANDING_SHLDR_V_INV+i);
    delay(15/speedMult);
    
    leg3[1].write(STANDING_SHLDR_V_REG-i);
    delay(15/speedMult);
    
    leg4[1].write(UNRAISED_INV_LEG-i);
    delay(15/speedMult);
  }
  // Leg
  for (int i = 3; i <= 35; i+= 3)
  {
    leg1[2].write(COMPRESSED_REG_LEG+i);
    delay(15/speedMult);
    
    if (i < 15) leg2[2].write(STANDING_LEG_INV-i);
    else leg2[2].write(EXTENDED_INV_LEG);
    delay(15/speedMult);
    
    leg3[2].write(STANDING_LEG_REG-i);
    delay(15/speedMult);
    
    if (i < 15) leg4[2].write(EXTENDED_INV_LEG+i);
    else leg4[2].write(STANDING_LEG_INV);
    delay(15/speedMult);
  }
  
  // Undoing extension of leg 3
  unextendRegLeg(leg3);
  lastStepRight = false;
}

// Makes the robot take a step forward with the left leg
void leftStepForward()
{
  extendInvLeg(leg4);

  // Turning the shoulders to drag the robot forward
  for (int i = 3; i <= 55; i+= 3)
  {
    leg3[0].write(STANDING_SHLDR_REG-i);
    delay(10/speedMult);
    
    leg1[0].write(COMPRESSED_SHLDR_REG+i);
    delay(10/speedMult);
    
    if (i < 21) leg4[0].write(EXTENDED_SHLDR_INV+i);
    else leg4[0].write(STANDING_SHLDR_INV);
    delay(10/speedMult);
    
    if (i < 21) leg2[0].write(STANDING_SHLDR_INV-i);
    else leg2[0].write(EXTENDED_SHLDR_INV);
    delay(10/speedMult);
  }

  // Adjusting the legs to be in the proper position
  // Vertical shoulder
  for (int i = 1; i <= 5; i++)
  {
    leg1[1].write(UNRAISED_REG_LEG+i);
    delay(15/speedMult);
    
    leg2[1].write(STANDING_SHLDR_V_INV+i);
    delay(15/speedMult);
    
    leg3[1].write(STANDING_SHLDR_V_REG-i);
    delay(15/speedMult);
    
    leg4[1].write(UNRAISED_INV_LEG-i);
    delay(15/speedMult);
  }
  // Leg
  for (int i = 3; i <= 35; i+= 3)
  {
    if (i < 15) leg1[2].write(EXTENDED_REG_LEG-i);
    else leg1[2].write(STANDING_LEG_REG);
    delay(15/speedMult);
    
    leg2[2].write(STANDING_LEG_INV+i);
    delay(15/speedMult);
    
    if (i < 15) leg3[2].write(STANDING_LEG_REG+i);
    else leg3[2].write(EXTENDED_REG_LEG);
    delay(15/speedMult);
    
    leg4[2].write(COMPRESSED_INV_LEG-i);
    delay(15/speedMult);
  }

  // Preparing leg 2 for the next step
  // Raising the leg
  for (int i = UNRAISED_INV_LEG; i >= RAISED_INV_LEG; i-= 2)
  {
    leg2[1].write(i);
    delay(15/speedMult);
  }
  // Getting into position
  for (int i = EXTENDED_SHLDR_INV; i <= COMPRESSED_SHLDR_INV; i+= 3)
  {
    leg2[0].write(i);
    delay(15/speedMult);
  }
  for (int i = COMPRESSED_INV_LEG; i >= EXTENDED_INV_LEG; i-= 3)
  {
    leg2[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_INV_LEG; i <= UNRAISED_INV_LEG; i+= 2)
  {
    leg2[1].write(i);
    delay(15/speedMult);
  }

  lastStepRight = false;
}

// Makes the robot take a step back with the right leg
void rightStepBack()
{
  // Undoing preparation of leg 2
  // Raising the leg
  for (int i = UNRAISED_INV_LEG; i >= RAISED_INV_LEG; i-= 2)
  {
    leg2[1].write(i);
    delay(15/speedMult);
  }
  // Getting into position
  for (int i = COMPRESSED_SHLDR_INV; i >= EXTENDED_SHLDR_INV; i-= 3)
  {
    leg2[0].write(i);
    delay(15/speedMult);
  }
  for (int i = EXTENDED_INV_LEG; i <= COMPRESSED_INV_LEG; i+= 3)
  {
    leg2[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_INV_LEG; i <= UNRAISED_INV_LEG; i+= 2)
  {
    leg2[1].write(i);
    delay(15/speedMult);
  }

  // Turning the shoulders to drag the robot backwards
  for (int i = 3; i <= 55; i+= 3)
  {
    leg3[0].write(COMPRESSED_SHLDR_REG+i);
    delay(10/speedMult);
    
    leg1[0].write(STANDING_SHLDR_REG-i);
    delay(10/speedMult);
    
    if (i < 21) leg4[0].write(STANDING_SHLDR_INV-i);
    else leg4[0].write(EXTENDED_SHLDR_INV);
    delay(10/speedMult);
    
    if (i < 21) leg2[0].write(EXTENDED_SHLDR_INV+i);
    else leg2[0].write(STANDING_SHLDR_INV);
    delay(10/speedMult);
  }

  // Adjusting the legs to be in the proper position
  // Vertical shoulder
  for (int i = 1; i <= 5; i++)
  {
    leg1[1].write(STANDING_SHLDR_V_REG-i);
    delay(15/speedMult);
    
    leg2[1].write(UNRAISED_INV_LEG-i);
    delay(15/speedMult);
    
    leg3[1].write(UNRAISED_REG_LEG+i);
    delay(15/speedMult);
    
    leg4[1].write(STANDING_SHLDR_V_INV+i);
    delay(15/speedMult);
  }
  // Leg
  for (int i = 3; i <= 35; i+= 3)
  {
    if (i < 15) leg1[2].write(STANDING_LEG_REG+i);
    else leg1[2].write(EXTENDED_REG_LEG);
    delay(15/speedMult);
    
    leg2[2].write(COMPRESSED_INV_LEG-i);
    delay(15/speedMult);
    
    if (i < 15) leg3[2].write(EXTENDED_REG_LEG-i);
    else leg3[2].write(STANDING_LEG_REG);
    delay(15/speedMult);
    
    leg4[2].write(STANDING_LEG_INV+i);
    delay(15/speedMult);
  }
  
  // Undoing extension of leg 4
  unextendInvLeg(leg4);
  lastStepRight = true;
}

/*--------------- LEFT & RIGHT MOVEMENT ---------------*/
// Turns the robot right 30 degrees
void turnRight()
{
  // Getting the robot into position if not already so
  if (!isFirstStep)
  {
    if (lastStepRight) alignLeftLegs();
    else alignRightLegs();
  }
  
  // Turning all the shoulders to spin the robot clockwise
  for (int i = 2; i <= 30; i+= 2)
  {
    leg1[0].write(STANDING_SHLDR_REG+i);
    delay(10/speedMult);

    leg2[0].write(STANDING_SHLDR_INV+i);
    delay(10/speedMult);

    leg3[0].write(STANDING_SHLDR_REG+i);
    delay(10/speedMult);

    leg4[0].write(STANDING_SHLDR_INV+i);
    delay(10/speedMult);
  }

  // Readjusting legs to standing position
  realignRegLeg(leg1, true);
  realignRegLeg(leg3, true);
  realignInvLeg(leg2, true);
  realignInvLeg(leg4, true);

  isFirstStep = true;
}

// Turns the robot left 30 degrees
void turnLeft()
{
  // Getting the robot into position if not already so
  if (!isFirstStep)
  {
    if (lastStepRight) alignLeftLegs();
    else alignRightLegs();
  }

  // Turning all the shoulders to spin the robot counter clockwise
  for (int i = 2; i <= 30; i+= 2)
  {
    leg1[0].write(STANDING_SHLDR_REG-i);
    delay(10/speedMult);

    leg2[0].write(STANDING_SHLDR_INV-i);
    delay(10/speedMult);

    leg3[0].write(STANDING_SHLDR_REG-i);
    delay(10/speedMult);

    leg4[0].write(STANDING_SHLDR_INV-i);
    delay(10/speedMult);
  }

  // Readjusting legs to standing position
  realignRegLeg(leg1, false);
  realignRegLeg(leg3, false);
  realignInvLeg(leg2, false);
  realignInvLeg(leg4, false);

  isFirstStep = true;
}

/*--------------- ADJUSTMENT FUNCTIONS ---------------*/

// Special case handling for the first step the robot takes
void firstStep()
{
  // Raising the leg off the ground
  for (int i = STANDING_SHLDR_V_INV; i >= RAISED_INV_LEG; i-= 3)
  {
    leg2[1].write(i);
    delay(15/speedMult);
  }
  // Moving into position
  for (int i = STANDING_SHLDR_INV; i <= COMPRESSED_SHLDR_INV; i+= 2)
  {
    leg2[0].write(i);
    delay(15/speedMult);
  }
  for (int i = STANDING_LEG_INV; i >= EXTENDED_INV_LEG; i-= 3)
  {
    leg2[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_INV_LEG; i <= UNRAISED_INV_LEG; i+= 2)
  {
    leg2[1].write(i);
    delay(15/speedMult);
  }

  // Repeating for the other leg
  for (int i = STANDING_SHLDR_V_REG; i <= RAISED_REG_LEG; i+= 3)
  {
    leg3[1].write(i);
    delay(15/speedMult);
  }
  for (int i = STANDING_SHLDR_REG; i >= COMPRESSED_SHLDR_REG; i-= 2)
  {
    leg3[0].write(i);
    delay(15/speedMult);
  }
  for (int i = STANDING_LEG_REG; i <= EXTENDED_REG_LEG; i+= 3)
  {
    leg3[2].write(i);
    delay(15/speedMult);
  }
  for (int i = RAISED_REG_LEG; i >= UNRAISED_REG_LEG; i-= 2)
  {
    leg3[1].write(i);
    delay(15/speedMult);
  }
}

// Extends the right leg forward
void extendRegLeg(Servo leg[3])
{
  // Raising the leg
  for (int i = UNRAISED_REG_LEG; i <= RAISED_REG_LEG; i+= 2)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
  // Getting into position
  for (int i = COMPRESSED_SHLDR_REG; i <= EXTENDED_SHLDR_REG; i+= 3)
  {
    leg[0].write(i);
    delay(15/speedMult);
  }
  for (int i = EXTENDED_REG_LEG; i >= COMPRESSED_REG_LEG; i-= 3)
  {
    leg[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_REG_LEG; i >= UNRAISED_REG_LEG; i-= 2)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
}

// Undoes extension of the right leg
void unextendRegLeg(Servo leg[3])
{
  // Raising the leg
  for (int i = UNRAISED_REG_LEG; i <= RAISED_REG_LEG; i+= 2)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
  // Getting into position
  for (int i = EXTENDED_SHLDR_REG; i >= COMPRESSED_SHLDR_REG; i-= 3)
  {
    leg[0].write(i);
    delay(15/speedMult);
  }
  for (int i = COMPRESSED_REG_LEG; i <= EXTENDED_REG_LEG; i+= 3)
  {
    leg[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_REG_LEG; i >= UNRAISED_REG_LEG; i-= 2)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
}

// Extends the left leg forward
void extendInvLeg(Servo leg[3])
{
  // Raising the leg
  for (int i = UNRAISED_INV_LEG; i >= RAISED_INV_LEG; i-= 2)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
  // Getting into position
  for (int i = COMPRESSED_SHLDR_INV; i >= EXTENDED_SHLDR_INV; i-= 3)
  {
    leg[0].write(i);
    delay(15/speedMult);
  }
  for (int i = EXTENDED_INV_LEG; i <= COMPRESSED_INV_LEG; i+= 3)
  {
    leg[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_INV_LEG; i <= UNRAISED_INV_LEG; i+= 2)
  {
    leg[1].write(i);
    delay(15/speedMult);
  } 
}

// Undoes extension of left leg
void unextendInvLeg(Servo leg[3])
{
  // Raising the leg
  for (int i = UNRAISED_INV_LEG; i >= RAISED_INV_LEG; i-= 2)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
  // Getting into position
  for (int i = EXTENDED_SHLDR_INV; i <= COMPRESSED_SHLDR_INV; i+= 3)
  {
    leg[0].write(i);
    delay(15/speedMult);
  }
  for (int i = COMPRESSED_INV_LEG; i >= EXTENDED_INV_LEG; i-= 3)
  {
    leg[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_INV_LEG; i <= UNRAISED_INV_LEG; i+= 2)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
}

// Readying the robot for a turn after stepping forward/backward with the left leg
void alignRightLegs()
{
  // Raising the leg off the ground
  for (int i = UNRAISED_INV_LEG; i >= RAISED_INV_LEG; i-= 2)
  {
    leg2[1].write(i);
    delay(15/speedMult);
  }
  // Moving into position
  for (int i = COMPRESSED_SHLDR_INV; i >= STANDING_SHLDR_INV; i-= 2)
  {
    leg2[0].write(i);
    delay(15/speedMult);
  }
  for (int i = EXTENDED_INV_LEG; i <= STANDING_LEG_INV; i+= 3)
  {
    leg2[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_INV_LEG; i <= STANDING_SHLDR_V_INV; i+= 3)
  {
    leg2[1].write(i);
    delay(15/speedMult);
  }

  // Repeating for the other leg
  for (int i = UNRAISED_REG_LEG; i <= RAISED_REG_LEG; i+= 2)
  {
    leg3[1].write(i);
    delay(15/speedMult);
  }
  for (int i = COMPRESSED_SHLDR_REG; i <= STANDING_SHLDR_REG; i+= 2)
  {
    leg3[0].write(i);
    delay(15/speedMult);
  }
  for (int i = EXTENDED_REG_LEG; i >= STANDING_LEG_REG; i-= 3)
  {
    leg3[2].write(i);
    delay(15/speedMult);
  }
  for (int i = RAISED_REG_LEG; i >= STANDING_SHLDR_V_REG; i-= 3)
  {
    leg3[1].write(i);
    delay(15/speedMult);
  }
}

// Readying the robot for a turn after stepping forward/backward with the right leg
void alignLeftLegs()
{
  // Raising the leg off the ground
  for (int i = UNRAISED_INV_LEG; i >= RAISED_INV_LEG; i-= 2)
  {
    leg4[1].write(i);
    delay(15/speedMult);
  }
  // Moving into position
  for (int i = COMPRESSED_SHLDR_INV; i >= STANDING_SHLDR_INV; i-= 2)
  {
    leg4[0].write(i);
    delay(15/speedMult);
  }
  for (int i = EXTENDED_INV_LEG; i <= STANDING_LEG_INV; i+= 3)
  {
    leg4[2].write(i);
    delay(15/speedMult);
  }
  // Bringing the leg back to the ground
  for (int i = RAISED_INV_LEG; i <= STANDING_SHLDR_V_INV; i+= 3)
  {
    leg4[1].write(i);
    delay(15/speedMult);
  }

  // Repeating for the other leg
  for (int i = UNRAISED_REG_LEG; i <= RAISED_REG_LEG; i+= 2)
  {
    leg1[1].write(i);
    delay(15/speedMult);
  }
  for (int i = COMPRESSED_SHLDR_REG; i <= STANDING_SHLDR_REG; i+= 2)
  {
    leg1[0].write(i);
    delay(15/speedMult);
  }
  for (int i = EXTENDED_REG_LEG; i >= STANDING_LEG_REG; i-= 3)
  {
    leg1[2].write(i);
    delay(15/speedMult);
  }
  for (int i = RAISED_REG_LEG; i >= STANDING_SHLDR_V_REG; i-= 3)
  {
    leg1[1].write(i);
    delay(15/speedMult);
  }
}

// Realigns odd numbered leg after turning
void realignRegLeg(Servo leg[3], bool rightTurn)
{
  int turnMult;
  int startPoint;
  if (rightTurn) 
  {
    turnMult = 1;
    startPoint = 165;
  }
  else
  {
    turnMult = -1;
    startPoint = 105;
  }

  // Raising the leg
  for (int i = STANDING_SHLDR_V_REG; i <= RAISED_REG_LEG; i+= 3)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
  // Turning the shoulder back
  for (int i = startPoint; i != STANDING_SHLDR_REG; i-= 2*turnMult)
  {
    leg[0].write(i);
    delay(15/speedMult);
  }
  // Dropping the leg
  for (int i = RAISED_REG_LEG; i >= STANDING_SHLDR_V_REG; i-= 3)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
}

// Realigns even numbered leg after turning
void realignInvLeg(Servo leg[3], bool rightTurn)
{
  int turnMult;
  int startPoint;
  if (rightTurn) 
  {
    turnMult = 1;
    startPoint = STANDING_SHLDR_V_INV;
  }
  else
  {
    turnMult = -1;
    startPoint = 15;
  }

  // Raising the leg
  for (int i = STANDING_SHLDR_V_INV; i >= RAISED_INV_LEG; i-= 3)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
  // Turning the shoulder back
  for (int i = startPoint; i != STANDING_SHLDR_INV; i-= 2*turnMult)
  {
    leg[0].write(i);
    delay(15/speedMult);
  }
  // Dropping the leg
  for (int i = RAISED_INV_LEG; i <= STANDING_SHLDR_V_INV; i+= 3)
  {
    leg[1].write(i);
    delay(15/speedMult);
  }
}

/*--------------- OLED FUNCTIONS ---------------*/

void cierra() {
  display.clearDisplay();
  display.drawFastHLine(40, 15, 20, WHITE);
  display.drawFastHLine(72, 20, 15, WHITE);
  display.display();
}

void triste() {
  display.clearDisplay();
  display.fillCircle (42, 10, 17, WHITE); //ojo izquierdo grande
  display.fillCircle (82, 10, 17, WHITE); //ojo derecho pequeño
  display.fillTriangle (0, 0, 0, 35, 78, 0, BLACK); //ceja superior
  display.fillTriangle (50, 0, 128, 35, 128, 0, BLACK); //ceja superior
  display.display();
}

void happy() {
  display.clearDisplay();
  display.fillCircle (42, 25, 15, WHITE); //ojo izquierdo grande
  display.fillCircle (82, 25, 15, WHITE); //ojo derecho pequeño
  display.fillCircle (42, 33, 20, BLACK); //ojo izquierdo grande
  display.fillCircle (82, 33, 20, BLACK); //ojo derecho pequeño
  display.display();
}

void enfado() {
  display.clearDisplay();
  display.fillCircle (42, 10, 18, WHITE); //ojo izquierdo grande
  display.fillCircle (82, 10, 12, WHITE); //ojo derecho pequeño
  display.fillTriangle (0, 0, 54, 26, 118, 0, BLACK); //ceja superior
  display.display();
}

void enfado1() {
  display.clearDisplay();
  display.fillCircle (42, 10, 18, WHITE); //ojo izquierdo grande
  display.fillCircle (82, 10, 12, WHITE); //ojo derecho pequeño
  display.fillTriangle (0, 0, 65, 15, 120, 0, BLACK); //ceja superior
  display.display();
}
