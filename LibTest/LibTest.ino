#include <SoftwareSerial.h>

#include <dynamixel.h> //Own library

dynamixel motor1(1); //Initiate motor
dynamixel motor2(2);
dynamixel motor3(3);
dynamixel motor4(4);
dynamixel motor5(5);

dynamixel motors[] = {motor1, motor2};
void setup()
{

  Serial.begin(57600); //Set baud rate
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  //Different commandos
  //All commandos shown below in setup/loop
  motor1.changeMode(1);
  motor2.changeMode(3);
  motor3.changeMode(3);
  motor4.changeMode(3);
  motor5.changeMode(3);
  motor1.action();
  motor2.action();
  motor3.action();
  motor4.action();
  motor5.action();
  motor3.pwmLimit(200);
  motor3.action();
  motor1.velLimit(100);
  motor1.action();
  motor2.torgueEnable();
  motor3.torgueEnable();
  motor4.torgueEnable();
  motor2.action();
  motor3.action();
  motor4.action();
  motor2.goalPos(230);
  motor3.goalPos(131);
  motor4.goalPos(182);
  motor2.action();
  motor3.action();
  motor4.action();
  motor1.torgueEnable();
  motor1.action();
  motor4.torgueDisable()
  motor4.action();
  //motor4 keep between 87 and 182 
  //motor3 HOME 131 degree
  //motor2 HOME 230 degree
}

void loop()
{
  motor1.goalVel(50);
  motor1.action();
  motor4.ledOn();
  motor4.action();
  delay(1000);
  motor1.goalVel(-50);
  motor1.action();
   motor4.ledOff();
  motor4.action();
  delay(1000);

}
