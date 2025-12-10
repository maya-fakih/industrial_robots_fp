// =====================================
// FINAL WORKING VERSION WITH PYTHON MODE
// =====================================

#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>

SoftwareSerial BT(10, 11); 
Adafruit_PWMServoDriver Servos_Drive = Adafruit_PWMServoDriver();

#define SERVO_MIN 102 
#define SERVO_MAX 512 

volatile long base_angle = 95;
volatile long motor2_angle = 90;
volatile long motor3_angle = 110;
volatile long motor4_angle = 180;
volatile long motor5_angle = 130;
volatile long motor6_angle = 70;

uint8_t BaseServo_Pos = 0;
uint8_t Motor2_Pos = 3;
uint8_t Motor3_Pos = 4;
uint8_t Motor4_Pos = 7;
uint8_t Motor5_Pos = 12;
uint8_t Motor6_Pos = 15;

#define Base_Servo_LEFT 'L'
#define Base_Servo_RIGHT 'R'
#define MOTOR2_UP 'U'
#define MOTOR2_DN 'D'
#define MOTOR34_DN 'B'
#define MOTOR34_UP 'F'
#define GRIPPERMOTOR_ROTATELEFT 'S'
#define GRIPPERMOTOR_ROTATERIGHT 'Y'
#define OPEN_GRIPPER 'O'
#define CLOSE_GRIPPER 'C'

int Delta_Angle = 3;
int Gripper_Delta_Angle = 5;
char command;

#define Save_JointVariables_Btn 18 
#define Execute_ActionsSequence 19 

// =========================
// MAX 10 POSITIONS
// =========================
volatile long RobotArmPositions[10][6];
volatile int Position_Indexer = 0;

volatile bool SavePosition_Flag = false;
volatile bool ExecuteSequence_Flag = false;

char serialLineBuf[128];
uint8_t serialLineIdx = 0;

// PYTHON MODE FLAG
volatile bool PYTHON_MODE = false;

// ISR flags only
void Save_JointVariables_ISR() {
  SavePosition_Flag = true;
}

void ExecuteSequence_ISR() {
  ExecuteSequence_Flag = true;
}

void setup() 
{
  Wire.begin();
  Servos_Drive.begin();
  Servos_Drive.setOscillatorFrequency(25000000);
  Servos_Drive.setPWMFreq(50);

  BT.begin(9600);
  Serial.begin(115200);

  pinMode(Save_JointVariables_Btn, INPUT_PULLUP);
  pinMode(Execute_ActionsSequence, INPUT_PULLUP); 

  attachInterrupt(digitalPinToInterrupt(Save_JointVariables_Btn), Save_JointVariables_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(Execute_ActionsSequence), ExecuteSequence_ISR, RISING);

  InitializeMotor_Postions();
}

void loop() 
{
  readFromBluetooth();
  readFromPythonNonBlocking();

  // ----------------------
  // FIXED SAVE FLAG LOGIC
  // ----------------------
  if (SavePosition_Flag)
  {
    SavePosition_Flag = false; 
    SaveArmPosition();
  }

  // -------------------------
  // FIXED EXECUTE FLAG LOGIC
  // -------------------------
  if (ExecuteSequence_Flag)
  {
    ExecuteSequence_Flag = false;
    ExecuteSequence();
  }
}

void readFromBluetooth()
{
  if (PYTHON_MODE) return;  // Ignore BT while in Python mode

  if (BT.available())
  {
    command = BT.read();
    executeBluetoothCommand(command);
  }
}

void readFromPythonNonBlocking() 
{
  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n' || serialLineIdx >= (sizeof(serialLineBuf)-1)) {
      serialLineBuf[serialLineIdx] = '\0';
      if (serialLineIdx > 0) {
        processPythonLine(serialLineBuf);
      }
      serialLineIdx = 0;
    } else {
      serialLineBuf[serialLineIdx++] = c;
    }
  }
}

void processPythonLine(const char* lineCStr)
{
  // ----- Mode switching -----
  if (strcmp(lineCStr, "MODE,PY") == 0) {
    PYTHON_MODE = true;
    Serial.println("Switched to Python mode");
    return;
  }

  if (strcmp(lineCStr, "MODE,BT") == 0) {
    PYTHON_MODE = false;
    Serial.println("Switched to Bluetooth/manual mode");
    return;
  }

  if (strcmp(lineCStr, "CMD,SAVE") == 0) {
    SavePosition_Flag = true;
    Serial.println("Python triggered: Save Position");
    return;
  }

  if (strcmp(lineCStr, "CMD,EXEC") == 0) {
      ExecuteSequence_Flag = true;
      Serial.println("Python triggered: Execute Sequence");
      return;
  }

  String line = String(lineCStr);
  line.trim();

  if (line.length() < 3) {
    Serial.println("No Joint Variable Angles were provided. No execution.");
    return;
  }

  char Angle_Check = line.charAt(0);
  if (line.charAt(1) != ',') {
    Serial.println("Malformed input.");
    return;
  }

  String data = line.substring(2);

  float values[6] = {0};
  int idx = 0;
  int start = 0;
  int len = data.length();
  for (int i = 0; i < len && idx < 6; i++) {
    if (data.charAt(i) == ',') {
      String token = data.substring(start, i);
      values[idx++] = token.toFloat();
      start = i + 1;
    } else if (i == len - 1) {
      String token = data.substring(start, len);
      values[idx++] = token.toFloat();
    }
  }

  if (idx != 6) {
    Serial.print("Expected 6 joint values, got ");
    Serial.println(idx);
    return;
  }

  if (Angle_Check == 'T') {
    for (int i = 0; i < 6; i++)
      values[i] = values[i] * 180.0 / PI;
  }

  noInterrupts();
  base_angle = (long)values[0];
  motor2_angle = (long)values[1];
  motor3_angle = (long)values[2];
  motor4_angle = (long)values[3];
  motor5_angle = (long)values[4];
  motor6_angle = (long)values[5];
  interrupts();

  Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(base_angle));
  Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(motor2_angle));
  Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(motor3_angle));
  Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(motor4_angle));
  Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(motor5_angle));
  Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(motor6_angle));

  Serial.print("Joint Variable Angles Executed: ");
  Serial.println(data);
}

// ----------------------
// Original functions below remain exactly as before
// angleToPulse, InitializeMotor_Postions, ActuateMotors_Positions,
// executeBluetoothCommand, SaveArmPosition, ExecuteSequence
// ----------------------

uint16_t angleToPulse(long angle) 
{
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void InitializeMotor_Postions()
{
  Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(95));
  Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(90));
  Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(110));
  Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(180));
  Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(130));
  Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(70));

  Serial.println("Motors initialized.");
}

void ActuateMotors_Positions(volatile long arr[10][6], int i)
{
  Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(arr[i][0]));
  Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(arr[i][1]));
  Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(arr[i][2]));
  Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(arr[i][3]));
  Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(arr[i][4]));
  Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(arr[i][5]));
}

void executeBluetoothCommand(char command)
{
  switch (command) 
  {
    case Base_Servo_LEFT:
      base_angle = constrain(base_angle + Delta_Angle, 0, 180);
      Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(base_angle));
      break;

    case Base_Servo_RIGHT:
      base_angle = constrain(base_angle - Delta_Angle, 0, 180);
      Servos_Drive.setPWM(BaseServo_Pos, 0, angleToPulse(base_angle));
      break;

    case MOTOR2_UP:
      motor2_angle = constrain(motor2_angle - Delta_Angle, 0, 180);
      Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(motor2_angle));
      break;

    case MOTOR2_DN:
      motor2_angle = constrain(motor2_angle + Delta_Angle, 0, 180);
      Servos_Drive.setPWM(Motor2_Pos, 0, angleToPulse(motor2_angle));
      break;

    case MOTOR34_DN:
      motor3_angle = constrain(motor3_angle + Delta_Angle, 0, 180);
      motor4_angle = constrain(motor4_angle + Delta_Angle, 0, 180);
      Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(motor3_angle));
      Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(motor4_angle));
      break;

    case MOTOR34_UP:
      motor3_angle = constrain(motor3_angle - Delta_Angle, 0, 180);
      motor4_angle = constrain(motor4_angle - Delta_Angle, 0, 180);
      Servos_Drive.setPWM(Motor3_Pos, 0, angleToPulse(motor3_angle));
      Servos_Drive.setPWM(Motor4_Pos, 0, angleToPulse(motor4_angle));
      break;

    case GRIPPERMOTOR_ROTATELEFT:
      motor5_angle = constrain(motor5_angle - Delta_Angle, 0, 180);
      Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(motor5_angle));
      break;

    case GRIPPERMOTOR_ROTATERIGHT:
      motor5_angle = constrain(motor5_angle + Delta_Angle, 0, 180);
      Servos_Drive.setPWM(Motor5_Pos, 0, angleToPulse(motor5_angle));
      break;

    case OPEN_GRIPPER:
      InitializeMotor_Postions();
      break;

    case CLOSE_GRIPPER:
      motor6_angle = constrain(motor6_angle + Gripper_Delta_Angle, 0, 180);
      Servos_Drive.setPWM(Motor6_Pos, 0, angleToPulse(motor6_angle));
      break;
  }
}

void SaveArmPosition()
{
  if (Position_Indexer >= 10) {
    Serial.println("List full (10). Save ignored.");
    return;
  }

  RobotArmPositions[Position_Indexer][0] = base_angle;
  RobotArmPositions[Position_Indexer][1] = motor2_angle;
  RobotArmPositions[Position_Indexer][2] = motor3_angle;
  RobotArmPositions[Position_Indexer][3] = motor4_angle;
  RobotArmPositions[Position_Indexer][4] = motor5_angle;
  RobotArmPositions[Position_Indexer][5] = motor6_angle;

  Serial.print("Saved position #");
  Serial.println(Position_Indexer + 1);

  Position_Indexer++;
}

void ExecuteSequence()
{
  if (Position_Indexer == 0) {
    Serial.println("No saved positions.");
    return;
  }

  Serial.print("Executing ");
  Serial.print(Position_Indexer);
  Serial.println(" positions");

  for (int i = 0; i < Position_Indexer; i++)
  {
    Serial.print("Going to position ");
    Serial.println(i + 1);

    ActuateMotors_Positions(RobotArmPositions, i);
    delay(2000);
  }

  Serial.println("Sequence complete.");
  Position_Indexer = 0;
}
