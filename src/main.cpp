#include <Arduino.h>
/**
 * Hardware pin defines
 */
#define BOARD UKMARSBOT_V1
const int ENCODER_LEFT_CLK = 2;
const int ENCODER_RIGHT_CLK = 3;
const int ENCODER_LEFT_B = 4;
const int ENCODER_RIGHT_B = 5;
const int MOTOR_LEFT_DIR = 7;
const int MOTOR_RIGHT_DIR = 8;
const int MOTOR_LEFT_PWM = 9;
const int MOTOR_RIGHT_PWM = 10;
const int LED_RIGHT = 6;
const int LED_LEFT = 11;
const int EMITTER = 12;
const int SENSOR_0 = A0;
const int SENSOR_1 = A1;
const int SENSOR_2 = A2;
const int SENSOR_3 = A3;
const int SENSOR_4 = A4;
const int SENSOR_5 = A5;
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;
/****/

/***
 * Global variables
 */

uint32_t updateTime;
uint32_t updateInterval = 200;  // in milliseconds

const float MAX_MOTOR_VOLTS = 6.0f;
const float batteryDividerRatio = 2.0f;
float gBatteryVolts;

bool botRunning = false;
bool passedStartLine = false;
bool fullMarker = false;
const float MOTOR_SPEED = 2.0;
const int SENSOR_CHANNELS = 4;

const float LINE_WIDTH = 19.0;  // ADJUST THIS so that CTE is roughly equal to the error in mm
const float LINE_DETECT_THRESHOLD = 900.0;  // minimum value to register the line - ADJUST TO SUIT
const float LEFT_MARKER_THRESHOLD = 450.0;  // minimum value to register the turn marker
const float RIGHT_MARKER_THRESHOLD = 450.0; // minimum value to register the start marker
volatile float gSensorStart;
volatile float gSensorTurn;
volatile float gSensorRight;
volatile float gSensorLeft;

volatile float gSensorSum;
volatile float gSensorDifference;
volatile float gSensorCTE;

float getBatteryVolts() {
  int adcValue = analogRead(BATTERY_VOLTS);
  gBatteryVolts = adcValue * (5.0f * batteryDividerRatio / 1023.0f);
  return gBatteryVolts;
}

int decodeFunctionSwitch(int functionValue) {
  /**
   * Typical ADC values for all function switch settings
   */
  const int adcReading[] = {660, 647, 630, 614, 590, 570, 545, 522, 461,
                            429, 385, 343, 271, 212, 128, 44,  0};

  if (functionValue > 1000) {
    return 16;  // pushbutton closed
  }
  int result = 16;
  for (int i = 0; i < 16; i++) {
    if (functionValue > (adcReading[i] + adcReading[i + 1]) / 2) {
      result = i;
      break;
    }
  }
  return result;
}

int getFunctionSwitch() {
  int functionValue = analogRead(FUNCTION_PIN);
  Serial.print(F("  Function value: "));
  Serial.print(functionValue);
  int function = decodeFunctionSwitch(functionValue);
  return function;
}

void motorSetup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  digitalWrite(MOTOR_LEFT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR, 0);
  digitalWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_RIGHT_DIR, 0);
}

void setLeftMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_LEFT_DIR, 0);
    analogWrite(MOTOR_LEFT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, 1);
    analogWrite(MOTOR_LEFT_PWM, pwm);
  }
}

void setRightMotorPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm < 0) {
    digitalWrite(MOTOR_RIGHT_DIR, 0);
    analogWrite(MOTOR_RIGHT_PWM, -pwm);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, 1);
    analogWrite(MOTOR_RIGHT_PWM, pwm);
  }
}

void setMotorPWM(int left, int right) {
  setLeftMotorPWM(left);
  setRightMotorPWM(right);
}

void setLeftMotorVolts(float volts) {
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setLeftMotorPWM(motorPWM);
}

void setRightMotorVolts(float volts) {
  volts = constrain(volts, -MAX_MOTOR_VOLTS, MAX_MOTOR_VOLTS);
  int motorPWM = (int)((255.0f * volts) / gBatteryVolts);
  setRightMotorPWM(motorPWM);
}

void setMotorVolts(float left, float right) {
  setLeftMotorVolts(left);
  setRightMotorVolts(right);
}

void motorAction(int function) {
  switch (function) {
    case 0:
      setMotorVolts(0, 0);
      Serial.println("motors off");
      break;
    case 1:
      setMotorVolts(1.5, 1.5);
      Serial.println("forward 25%");
      break;
    case 2:
      setMotorVolts(3.0, 3.0);
      Serial.println("forward 50%");
      break;
    case 3:
      setMotorVolts(4.5, 4.5);
      Serial.println("forward 75%");
      break;
    case 4:
      setMotorVolts(-1.5, -1.5);
      Serial.println("reverse 25%");
      break;
    case 5:
      setMotorVolts(-3.0, -3.0);
      Serial.println("reverse 50%");
      break;
    case 6:
      setMotorVolts(-4.5, -4.5);
      Serial.println("reverse 75%");
      break;
    case 7:
      setMotorVolts(-1.5, 1.5);
      Serial.println("spin left 25%");
      break;
    case 8:
      setMotorVolts(-3.0, 3.0);
      Serial.println("spin left 50%");
      break;
    case 9:
      setMotorVolts(1.5, -1.5);
      Serial.println("spin right 25%");
      break;
    case 10:
      setMotorVolts(3.0, 3.0);
      Serial.println("spin right 50%");
      break;
    case 11:
      setMotorVolts(0, 1.5);
      Serial.println("pivot left 25%");
      break;
    case 12:
      setMotorVolts(1.5, 0);
      Serial.println("pivot right 25%");
      break;
    case 13:
      setMotorVolts(1.5, 3.0);
      Serial.println("curve left");
      break;
    case 14:
      setMotorVolts(3.0, 1.5);
      Serial.println("curve right");
      break;
    case 15:
      setMotorVolts(4.5, 3.0);
      Serial.println("big curve right");
      break;
    default:
      setMotorVolts(0, 0);
      break;
  }
}

void runRobot() {
  int function = getFunctionSwitch();
  // run the motors for a fixed amount of time (in milliseconds)
  uint32_t endTime = millis() + 2000;
 
  botRunning = true;
  fullMarker = false;
  setMotorVolts(1.5, 1.5);
  // motorAction(function);
  // while (endTime > millis()) {
  //   if (getFunctionSwitch() == 16) {
  //     break;  // stop running if the button is pressed
  //   }
  // }
  // be sure to turn off the motors
  // setMotorPWM(0, 0);
  // a short delay to let everything come to rest.
  // delay(500);
}

void analogueSetup() {
  // increase speed of ADC conversions to 28us each
  // by changing the clock prescaler from 128 to 16
  bitClear(ADCSRA, ADPS0);
  bitClear(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS2);
}

/***
 * If you are interested in what all this does, the ATMega328P datasheet
 * has all the answers but it is not easy to follow until you have some
 * experience. For now just use the code as it is.
 */
void setupSystick() {
  // set the mode for timer 2
  bitClear(TCCR2A, WGM20);
  bitSet(TCCR2A, WGM21);
  bitClear(TCCR2B, WGM22);
  // set divisor to 128 => timer clock = 125kHz
  bitSet(TCCR2B, CS22);
  bitClear(TCCR2B, CS21);
  bitSet(TCCR2B, CS20);
  // set the timer frequency
  OCR2A = 249;  // (16000000/128/500)-1 = 249
  // enable the timer interrupt
  bitSet(TIMSK2, OCIE2A);
}

void updateLineSensor() {
  // read ALL the sensor channels;
  int sensorDark[SENSOR_CHANNELS] = {0};
  int sensorLit[SENSOR_CHANNELS] = {0};
  int sensorValue[SENSOR_CHANNELS] = {0};
  for (int i = 0; i < SENSOR_CHANNELS; i++) {
    sensorDark[i] = analogRead(A0 + i);
  }
  digitalWrite(EMITTER, 1);
  delayMicroseconds(50);
  for (int i = 0; i < SENSOR_CHANNELS; i++) {
    sensorLit[i] = analogRead(A0 + i);
  }
  digitalWrite(EMITTER, 0);
  for (int i = 0; i < SENSOR_CHANNELS; i++) {
    sensorValue[i] = abs(sensorLit[i] - sensorDark[i]);
  }
  gSensorStart = sensorValue[0];
  gSensorRight = sensorValue[1];
  gSensorLeft = sensorValue[2];
  gSensorTurn = sensorValue[3];
  if (gSensorStart > RIGHT_MARKER_THRESHOLD) {
    digitalWrite(LED_RIGHT, 1);
  } else {
    digitalWrite(LED_RIGHT, 0);
  }
  if (gSensorTurn > LEFT_MARKER_THRESHOLD) {
    digitalWrite(LED_LEFT, 1);
  } else {
    digitalWrite(LED_LEFT, 0);
  }
  gSensorSum = gSensorRight + gSensorLeft;
  gSensorDifference = gSensorRight - gSensorLeft;
  if (gSensorSum > LINE_DETECT_THRESHOLD) {
    gSensorCTE = LINE_WIDTH * (gSensorDifference / gSensorSum);
  } else {
    gSensorCTE = 0;
  }
  if (botRunning) {
    if (gSensorCTE < 0) {
      setMotorVolts(MOTOR_SPEED + gSensorCTE, MOTOR_SPEED );
    }
    if (gSensorCTE > 0) {
      setMotorVolts(MOTOR_SPEED, MOTOR_SPEED - gSensorCTE);
    }

    if (gSensorStart > RIGHT_MARKER_THRESHOLD && gSensorTurn > LEFT_MARKER_THRESHOLD) {
      if (passedStartLine) {
        setMotorVolts(0, 0);
        botRunning = false; 
        passedStartLine = true;
      }
      fullMarker = true;

    } else {
      if (fullMarker) {
        passedStartLine = true;
        fullMarker = false;
      }
    }
  }
}
// the systick event is an ISR attached to Timer 2
ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  getBatteryVolts();  // update the battery reading
  updateLineSensor();
}

void setup() {
  Serial.begin(9600);
  pinMode(EMITTER, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  digitalWrite(EMITTER, 0);  // be sure the emitter is off
  analogueSetup();           // increase the ADC conversion speed
  setupSystick();
  motorSetup();
  updateTime = millis() + updateInterval;
}

void loop() {
  if (millis() > updateTime) {
    updateTime += updateInterval;
    Serial.print(F("  Start: "));
    Serial.print(gSensorStart);
    Serial.print(F("  Turn: "));
    Serial.print(gSensorTurn);
    Serial.print(F("  Right: "));
    Serial.print(gSensorRight);
    Serial.print(F("  Left: "));
    Serial.print(gSensorLeft);
    Serial.print(F("  Sum: "));
    Serial.print(gSensorSum);
    Serial.print(F("  Diff: "));
    Serial.print(gSensorDifference);
    Serial.print(F("  CTE: "));
    Serial.print(gSensorCTE);
    Serial.println();
  }
  if (getFunctionSwitch() == 16) {
    // button is pressed so wait until it is released
    while (getFunctionSwitch() == 16) {
      delay(20);
    }
    // now wait for the user to get clear
    delay(500);
    Serial.print(F("  Running robot! "));
    runRobot();
  }
}