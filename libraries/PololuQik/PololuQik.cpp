#include "PololuQik.h"

byte cmd[5]; // serial command buffer

PololuQik::PololuQik(unsigned char resetPin, unsigned char errorPin)
{
  _resetPin = resetPin;
  _errorPin = errorPin;
  errorFlag = false;
}

void PololuQik::init(long speed /* = 9600 */)
{
    
  pinMode(_errorPin, INPUT);
  digitalWrite(_errorPin, LOW);

  // reset the qik
  reset();
  
//  Serial1.end();
  Serial1.begin(speed);
  Serial1.write(0xAA); // allow qik to autodetect baud rate
}

void PololuQik::reset() {
    digitalWrite(_resetPin, LOW);
    pinMode(_resetPin, OUTPUT); // drive low
    delay(1);
    pinMode(_resetPin, INPUT); // return to high-impedance input (reset is internally pulled up on qik)
    delay(10);
}

bool PololuQik::isError() {
    return digitalRead(_errorPin);
}

char PololuQik::getFirmwareVersion()
{
  Serial1.write(QIK_GET_FIRMWARE_VERSION);
  while (Serial1.available() < 1);
  return Serial1.read();
}

byte PololuQik::getErrors()
{
  Serial1.write(QIK_GET_ERROR_BYTE);
  while (Serial1.available() < 1);
  return Serial1.read();
}

byte PololuQik::getConfigurationParameter(byte parameter)
{
  cmd[0] = QIK_GET_CONFIGURATION_PARAMETER;
  cmd[1] = parameter;
  Serial1.write(cmd, 2);
  while (Serial1.available() < 1);
  return Serial1.read();
}

byte PololuQik::setConfigurationParameter(byte parameter, byte value)
{
  
  cmd[0] = QIK_SET_CONFIGURATION_PARAMETER;
  cmd[1] = parameter;
  cmd[2] = value;
  cmd[3] = 0x55;
  cmd[4] = 0x2A;
  Serial1.write(cmd, 5);
  while (Serial1.available() < 1);
  return Serial1.read();
}

void PololuQik::setM0Speed(int speed)
{
  boolean reverse = 0;

  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1; // preserve the direction
  }

  if (speed > 255)
    speed = 255;

  if (speed > 127)
  {
    // 8-bit mode: actual speed is (speed + 128)
    cmd[0] = reverse ? QIK_MOTOR_M0_REVERSE_8_BIT : QIK_MOTOR_M0_FORWARD_8_BIT;
    cmd[1] = speed - 128;
  }
  else
  {
    cmd[0] = reverse ? QIK_MOTOR_M0_REVERSE : QIK_MOTOR_M0_FORWARD;
    cmd[1] = speed;
  }

  Serial1.write(cmd, 2);
}

void PololuQik::setM1Speed(int speed)
{
  boolean reverse = 0;

  if (speed < 0)
  {
    speed = -speed; // make speed a positive quantity
    reverse = 1; // preserve the direction
  }

  if (speed > 255)
    speed = 255;

  if (speed > 127)
  {
    // 8-bit mode: actual speed is (speed + 128)
    cmd[0] = reverse ? QIK_MOTOR_M1_REVERSE_8_BIT : QIK_MOTOR_M1_FORWARD_8_BIT;
    cmd[1] = speed - 128;
  }
  else
  {
    cmd[0] = reverse ? QIK_MOTOR_M1_REVERSE : QIK_MOTOR_M1_FORWARD;
    cmd[1] = speed;
  }
    
  Serial1.write(cmd, 2);
}

void PololuQik::setSpeeds(int m0Speed, int m1Speed)
{
  setM0Speed(m0Speed);
  setM1Speed(m1Speed);
}

// 2s12v10
void PololuQik2s12v10::setM0Brake(unsigned char brake)
{
  if (brake > 127)
      brake = 127;
  
  cmd[0] = QIK_2S12V10_MOTOR_M0_BRAKE;
  cmd[1] = brake;
  Serial1.write(cmd, 2);
}

void PololuQik2s12v10::setM1Brake(unsigned char brake)
{
  if (brake > 127)
      brake = 127;
  
  cmd[0] = QIK_2S12V10_MOTOR_M1_BRAKE;
  cmd[1] = brake;
  Serial1.write(cmd, 2);
}

void PololuQik2s12v10::setBrakes(unsigned char m0Brake, unsigned char m1Brake)
{
  setM0Brake(m0Brake);
  setM1Brake(m1Brake);
}

unsigned char PololuQik2s12v10::getM0Current()
{
  Serial1.write(QIK_2S12V10_GET_MOTOR_M0_CURRENT);
  while (Serial1.available() < 1);
  return Serial1.read();
}

unsigned char PololuQik2s12v10::getM1Current()
{
  Serial1.write(QIK_2S12V10_GET_MOTOR_M1_CURRENT);
  while (Serial1.available() < 1);
  return Serial1.read();
}

unsigned int PololuQik2s12v10::getM0CurrentMilliamps()
{
  return getM0Current() * 150;
}

unsigned int PololuQik2s12v10::getM1CurrentMilliamps()
{
  return getM1Current() * 150;
}

unsigned char PololuQik2s12v10::getM0Speed()
{
  Serial1.write(QIK_2S12V10_GET_MOTOR_M0_SPEED);
  while (Serial1.available() < 1);
  return Serial1.read();
}

unsigned char PololuQik2s12v10::getM1Speed()
{
  Serial1.write(QIK_2S12V10_GET_MOTOR_M1_SPEED);
  while (Serial1.available() < 1);
  return Serial1.read();
}
