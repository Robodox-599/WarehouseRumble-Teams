// danielle has claimed this territory 🚩

#include <Encoder.h>
#include <PS2X_lib.h>
#include <Servo.h>
#include "SPDMotor.h"
#include "Constants.h"

PS2X ps2x;
int error = 0;
byte type = 0;
int autoState = AUTOMODE_INIT;
int competitionMode = COMPMODE_STANDBY;
unsigned long timerStart = millis();
int placeholder;

void (* resetFunc) (void) = 0;  // whatevs


// motor initialization

Servo intakeMotor;
Servo outtakeMotor;
Servo elevatorMotor;
SPDMotor *motorLF = new SPDMotor(18, 31, true, 12, 34, 35); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 36, 37); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor(3, 49, true,  9, 43, 42);  // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor(2, A1, false, 5, A4, A5);  // <- NOTE: Motor Dir pins reversed for opposite operation


// ---------------------------------------------------------------------- //

void setup()
{
  Serial.begin(57600);
  delay(300);            // added delay to give wireless ps2 module some time to startup, before configuring it
  setupController();
  allStop();

  Serial.println("Going to Standby Mode");
}

void loop()
{
  switch (competitionMode)
  {
    case COMPMODE_STANDBY:
      ps2x.read_gamepad();
      if (ps2x.ButtonPressed(PSB_START))
      {
        Serial.println("Going to Auto Mode");
        competitionMode = COMPMODE_AUTO;
        autoState = AUTOMODE_INIT;
        timerStart = millis();
      }
      break;

    case COMPMODE_AUTO:
      doAutonomousLoop();
      if (millis() > (timerStart + TIMER_AUTONOMOUS_MS)) {
        Serial.println("Going to Teleop Mode");
        allStop();
        competitionMode = COMPMODE_TELEOP;
        timerStart = millis();
      }
      break;

    case COMPMODE_TELEOP:
      doTeleopLoop();
      if (millis() > (timerStart + TIMER_TELEOP_MS)) {
        Serial.println("Match over, going to Standby");
        allStop();
        competitionMode = COMPMODE_STANDBY;
      }
      break;
  }
  delay(20);
}

// ---------------------------------------------------------------------- //


void doTeleopLoop()
{
  static int teleopState = TELEOPMODE_NORMAL;
  static float intakeRollerSpeed = 0;

  if (error == 1) {
    return;  // skip loop if no controller found
  }
  if (type == 2) {
    return;  // Guitar Hero Controller
  }


  // We have a DualShock Controller that is responding

  ps2x.read_gamepad(); // read controller
  

  // DRIVING

  int LY = 255 - ps2x.Analog(PSS_LY);
  int LX = ps2x.Analog(PSS_LX);
  int RX = ps2x.Analog(PSS_RX);
  driveChassisJoystick(LY, LX, RX);

  
  if (ps2x.ButtonPressed(TELEOPMODE_SWITCH_BUTTON))
  {
    if (teleopState = TELEOPMODE_INTAKE_ELEVATOR) { teleopState = TELEOPMODE_NORMAL; stopMotors(); }
    else { teleopState = TELEOPMODE_INTAKE_ELEVATOR; stopMotors(); }
  }
  
  switch (teleopState)
  {
    case TELEOPMODE_NORMAL:    // button pressed --> outtake releases, pressed again --> outtake resets
      if(ps2x.ButtonPressed(TELEOPMODE_OUTTAKE_BUTTON)) { placeholder = 1; }
      if(ps2x.ButtonPressed(TELEOPMODE_OUTTAKE_BUTTON)) { placeholder = 0; }
      outtakeMotor.write(placeholder);
      break;

    case TELEOPMODE_INTAKE_ELEVATOR:
      // PSB_R1: increase speed by x amount using setIntakeRollerSpeed
      // PSB_L1: decrease speed by x amount using setIntakeRollerSpeed
      // Button --> continuous, ButtonPressed --> clicky
      if(ps2x.Button(ELEVATOR_UP) && ps2x.Button(ELEVATOR_DOWN)) { elevatorMotor.write(VEXSpeed(0)); }
      if(ps2x.Button(ELEVATOR_UP) && !ps2x.Button(ELEVATOR_DOWN)) { elevatorMotor.write(min(80, elevatorMotor.read() - 1)); }
      if(!ps2x.Button(ELEVATOR_UP) && ps2x.Button(ELEVATOR_DOWN)) { elevatorMotor.write(max(100, elevatorMotor.read() + 1)); }
      if(!ps2x.Button(ELEVATOR_UP) && !ps2x.Button(ELEVATOR_DOWN)) { elevatorMotor.write(VEXSpeed(0)); }
      // add clicky adjust
      break;
  }
}

void doAutonomousLoop()
{
  static int stateLoopCount = 0;
  static long encoderBookMark = 0;

  stateLoopCount++;

  // Autonomous Mode State Machine!!!1!11!!!
  switch (autoState)
  {
    case AUTOMODE_INIT:
      setNewState(AUTOMODE_FORWARD, stateLoopCount);
      break;

    case AUTOMODE_FORWARD:
      if (stateLoopCount == 1)
      {
        Serial.println("AUTOMODE_FORWARD");
        encoderBookMark = motorLF->getEncoderPosition();
        driveChassis(200, 0, 0);
      }

      if (abs(motorLF->getEncoderPosition() - encoderBookMark) > 1300)
      {
        Serial.println("DONE");
        driveChassis(0, 0, 0);
        setNewState(AUTOMODE_TURNLEFT, stateLoopCount);
      }
      break;

    case AUTOMODE_TURNLEFT:
      if (stateLoopCount == 1)
      {
        Serial.println("AUTOMODE_TURNLEFT");
        encoderBookMark = motorLF->getEncoderPosition();
        driveChassis(0, -200, 0);
      }

      if (abs(motorLF->getEncoderPosition() - encoderBookMark) > 1300)
      {
        Serial.println("DONE");
        driveChassis(0, 0, 0);
        setNewState(AUTOMODE_STANDBY, stateLoopCount);
      }
      break;

    case AUTOMODE_STANDBY:
      allStop();
      break;
  }
}

void driveChassisJoystick(int forRev, int turn, int slide)
{
  driveChassis((forRev * 2) - 255, (turn * 2) - 255, (slide * 2) - 255);
}

void driveChassis(int forRev, int turn, int slide)
{
  float forwardNormalized = (float)(forRev / 255.f);

  forwardNormalized = constrain( forwardNormalized, -1.f, 1.f );
  float multiplier = 255.f;
  int forward = (int)(pow(forwardNormalized, 2.0) * multiplier);

  // preserve the direction of movement
  if ( forwardNormalized < 0 ) {
    forward = -forward;
  }

  int right = -slide / 2;
  int ccwTurn = (turn / 4);

  motorLF->speed(forward + ccwTurn - right);
  motorRF->speed(forward - ccwTurn + right);
  motorLR->speed(forward + ccwTurn + right);
  motorRR->speed(-(forward - ccwTurn - right));
}

void setupController()
{

  // set up pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.println("Found Controller, configuration successful ");
    Serial.println();
    Serial.println("Left analog stick for forward/back and turning.");
    Serial.println("Right analog stick for sideways movement.");
    Serial.println("Hold both L1 and R1 for full-speed.");
  }
  else if (error == 1)
  {
    Serial.println("No controller found, check PS2 receiver is inserted the correct way around.");
    resetFunc();
  }
  else if (error == 2)
    Serial.println("Controller found but not accepting commands.");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type found ");
      break;
    case 1:
      Serial.println("DualShock Controller found ");
      break;
    case 2:
      Serial.println("GuitarHero Controller found ");
      break;
    case 3:
      Serial.println("Wireless Sony DualShock Controller found ");
      break;
  }
}

void allStop()
{
  driveChassis(0, 0, 0);
  stopMotors();
}

void stopMotors()
{
  intakeMotor.write(VEXSpeed(0));
  outtakeMotor.write(VEXSpeed(0));
  elevatorMotor.write(VEXSpeed(0));
}

void setNewState(int newState, int& stateLoopCount)
{
  autoState = newState;
  stateLoopCount = 0;
}

int VEXSpeed(float speed)
{
  if (speed < -1) { speed = -1; }
  if (speed > 1) { speed = 1; }

  return map(speed, -1, 1, 0, 180);
}