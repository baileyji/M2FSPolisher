#include <PololuQikOfficial.h>

#include <SoftwareSerial.h>

char* param_names[] = {
  "Device ID",
  "PWM Parameter",
  "Shut Down Motors on Error",
  "Serial Timeout",
  "Motor M0 Acceleration",
  "Motor M1 Acceleration",
  "Motor M0 Brake Duration",
  "Motor M1 Brake Duration",
  "Motor M0 Current Limit / 2",
  "Motor M1 Current Limit / 2",
  "Motor M0 Current-Limit Response",
  "Motor M1 Current-Limit Response"
};

PololuQik2s12v10 qik=PololuQik2s12v10(4, 3, 5);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  qik.init();


  for (int i = QIK_CONFIG_DEVICE_ID; i <= QIK_CONFIG_MOTOR_M1_CURRENT_LIMIT_RESPONSE; i++)
  {
    Serial.print(param_names[i]);
    Serial.print(": ");
    Serial.println(qik.getConfigurationParameter(i));
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
