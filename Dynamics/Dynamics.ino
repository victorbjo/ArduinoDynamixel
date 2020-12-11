#include <SoftwareSerial.h>
#include <dynamixel.h>
SoftwareSerial mySerial(10, 11);
dynamixel motor1(1); //Initiate motor
dynamixel motor2(2);
dynamixel motor3(3);
dynamixel motor4(4);
dynamixel motor5(5);

uint8_t c[3];
unsigned long StartTime;
void setup() {
  pinMode(12, OUTPUT);
  digitalWrite(12,HIGH);
  // put your setup code here, to run once:
  Serial.begin(57600);
  motor3.torgueDisable();
  motor3.action();
  motor3.pwmLimit(135); //Essentially speed of motor. play with this to get the wanted time <3
  motor3.action();
  motor3.changeMode(3); 
  motor3.torgueEnable();
  motor3.action();
  motor2.torgueEnable();
  motor2.action();
  motor3.goalPos(0); //If you wanna start at 0 deg. I recommend.
  motor3.action();
  delay(2000);
  StartTime = millis();
    motor3.goalPos(90); //End Deg. Do not go over 90 deg, otherwise It will probably fuck up a lotta data
  motor3.action();
}
int readPos(){
  digitalWrite(12,LOW);
    mySerial.begin(57600);
    delay(10);
    int i = 0;
    int count = 0;
  while (mySerial.available()){
    if (i == 9 || i ==10||i==11){
    c[count] = mySerial.read();
    count++;
    delay(1);
    }
    else{
      int a = mySerial.read();
    }
    i++;
  }
  digitalWrite(12,HIGH);
  uint16_t combined;
  

  //Serial.println(c[2]);
  int result;
    if (c[0] != 0){
    result = (c[1] * 256u) + c[0];
    result = float(result)*3.36;}
  return result;
}
float readVel(){
  digitalWrite(12,LOW);
    mySerial.begin(57600);
    delay(10);
    int i = 0;
    int count = 0;
  while (mySerial.available()){
    if (i == 9 || i ==10||i==11){
    c[count] = mySerial.read();
    count++;
    delay(1);
    }
    else{
      int a = mySerial.read();
    }
    i++;
  }
  digitalWrite(12,HIGH);
  uint16_t combined;
  

  //Serial.println(c[2]);
  float result;
    if (c[0] != 0){
    result = (c[1] * 256u) + c[0];
    result = float(result)*0.23;}
  return result;}
void loop() {

  motor3.getVel();
   float test =  readVel();
   if (test > 1000){
    test = 0;
   }
      motor3.getAmp();
  int test1 =  readPos();
  Serial.print("\n Amps:, ");
  Serial.print(test1);
  Serial.print(",");
  Serial.print("Vel:,");
  Serial.print(test);
  Serial.print(",");
  Serial.print("Time:,");
  Serial.print(millis()-StartTime);
  Serial.print(",");
   
  // put your main code here, to run repeatedly:
  /*uint8_t test1 = 0x00;
  uint8_t test2 = 0x76;
  int test;
  if (test1 > 0xf1){
  test = test1-test2+1;
  test = - test;
  }else{
    test = test1+test2;
  }*/
  //Serial.println(test);
  
}
