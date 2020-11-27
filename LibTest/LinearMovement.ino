#include <SoftwareSerial.h>
#include <dynamixel.h>
SoftwareSerial mySerial(10, 11);
dynamixel motor1(1); //Initiate motor
dynamixel motor2(2);
dynamixel motor3(3);
dynamixel motor4(4);
dynamixel motor5(5);
long l1 = 220;
long l2 = 270;
int c[3];
int ekstra = 4;
int length = l1+l2;
float x = 0;
float y;
int vel = 9;
int incomingByte = 0;
void setup() {
    Serial.begin(57600);
    pinMode(12, OUTPUT);
    digitalWrite(12,HIGH);
    // put your setup code here, to run once:
    motor1.changeMode(1);
    motor2.changeMode(1);
    motor3.changeMode(1);
    motor4.changeMode(1);
    motor5.changeMode(1);
    motor1.action();
    motor2.action();
    motor3.action();
    motor4.action();
    motor5.action();
    motor4.torgueEnable();
    motor5.torgueEnable();
    motor4.action();
    motor5.action();
    motor4.goalPos(182);
    motor4.action();
    motor2.torgueDisable();
    motor2.action();
    motor2.changeMode(1);
    motor2.action();
    motor2.torgueEnable();
    motor1.torgueEnable();
    motor1.action();
    motor2.action();
    motor3.torgueEnable();
    motor2.torgueEnable();
    motor3.action();
    motor2.action();
    //motor3.goalVel(-1);
    //motor3.action();
    int newTheta = getTheta(300, 300);
    int newTheta2 = getTheta2(300, 300);

    motor2.goalPos(newTheta+90);
    motor3.goalPos(newTheta2+90);
    motor2.action();
    motor3.action();
    delay(3001);

    
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
  if (c[2] == 0){
  int b = (c[0]-1)*10+c[1]+c[2];
  combined = (c[1] * 256u) + c[0]; }
  else{
  combined = (c[2] * 256u) + c[1]; 
  }
  int result = (double(float(360)/float(4096))*combined);
  return result;
}

bool isPossible(double x, double y){
    if (y < -210){
      return false;
    }
    int point = sqrt((x*x)+(y*y));
    return (length >= point+10);
}

int getTheta2(float x, float y){
    float beta = atan2(y,x);
    beta = beta * 57296 / 1000;
    //Serial.println(beta);
    double theta2 = acos((x*x+y*y-l1*l1-l2*l2)/(long(2)*l1*l2));
    theta2 = theta2 *57296 / 1000;
      if (abs(theta2)-abs(int(theta2))<= 0.5){
    if (theta2 < 0){
      theta2 = theta2 - 1;
    }else{
      theta2 = theta2 + 1;  
    }
    
  }
    return theta2;
    
}


int getTheta(float x, float y){
  float beta = atan2(y,x);
  beta = beta * 57296 / 1000; 
  double theta2 = acos((x*x+y*y-l1*l1-l2*l2)/(long(2)*l1*l2));
  theta2 = theta2 *57296 / 1000; //Calculates theta2 locally instead of using function 
  //since all variables are already set
  double b = sqrt(l2*l2+l1*l1-2*l2*l1*cos((180-theta2)*1000 / 57296));  //Calculates length b
  double theta = beta + 57296/1000*acos((b*b+l1*l1-l2*l2)/(2*b*l1));
  if (abs(theta)-abs(int(theta))<= 0.5){ //Rounds up theta if decimal is higher or equal to 0.5 
    if (theta < 0)
    {
      theta = theta;
    }
    else
    {
      theta = theta + 1;  
    } 
  }
  return theta;
}

double xPos(double theta1, double theta2){
  return l1*cos(theta1*1000/57296)+l2*cos((theta1-theta2)*1000 / 57296);
}

double yPos(double theta1, double theta2){
   return l1*sin(theta1*1000/57296)+l2*sin((theta1-theta2)*1000 / 57296);
}

void moveUp(){
  delay(10);
  motor2.getPos();
  int theta = readPos()-90;
  motor3.getPos();
  int theta2 = readPos()-90;
  float goalX = x; //xPos(theta, theta2);
  float OGx = x;
  float y = yPos(theta, theta2);
  Serial.println(theta);
  float actualX = xPos(theta, theta2);
  y += ekstra;
  if (actualX < x){
    goalX = x+2;
  }
  else if(actualX>goalX){
    goalX = x-2;
  }
  int newTheta = getTheta(goalX, y);
  int newTheta2 = getTheta2(goalX, y);
  if (isPossible(x,y)==false){
    Serial.print("impossible position");
    Serial.print(x);
    Serial.print(" : ");
    Serial.println(y);
    return;
  }
  Serial.println(isPossible(x,y));
    
    Serial.print("\n X deviation:");
    
    Serial.print(OGx - actualX);
    Serial.print("\n Y");
    Serial.print(y);
    Serial.println(theta2);
    Serial.println(theta);
    Serial.println(newTheta2);
    Serial.println(newTheta);
    Serial.println();
    motor2.goalPos(newTheta+90);
    motor3.goalPos(newTheta2+90);
    motor2.action();
    motor3.action();
  
}
void openGrippers(){

    motor4.goalVel(5);
    motor4.action();
    motor5.goalVel(-5);
    motor5.action();
}
void closeGrippers(){

    motor4.goalVel(-5);
    motor4.action();
    motor5.goalVel(5);
    motor5.action();
}
void moveAlong(){
  motor2.getPos();
  int theta = readPos()-90;
  motor3.getPos();
  int theta2 = readPos()-90;
  float goalY = y; //xPos(theta, theta2);
  float OGy = y;
  float x = xPos(theta, theta2);
  float actualY = yPos(theta, theta2);
  x += ekstra;
  if (actualY < y){
    goalY = y+2;
  }
  else if(actualY>goalY){
    goalY = y-2;
  }
  if (actualY>goalY+4){
    goalY = y-6;
  }
  else if (actualY+4 < y){
    goalY = y+6;
  }
  int newTheta = getTheta(x, goalY);
  int newTheta2 = getTheta2(x, goalY);
  if (isPossible(x,y)==false){
    return;
  }/*
  Serial.println(isPossible(x,y));
    
   
    Serial.print("\n X: ");
    Serial.print(x);
    Serial.print("\n Y deviation:");
    Serial.println(OGy - actualY);
    Serial.println(actualY);*/
    motor2.goalPos(newTheta+90);
    motor3.goalPos(newTheta2+90);
    motor2.action();
    motor3.action();  
}
void loop() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
  }
  switch (incomingByte){
    case 1:
      motor1.goalVel(0);
      motor1.action();
      motor2.goalVel(0);
      motor2.action();
      motor3.goalVel(0);
      motor3.action();
      motor4.goalVel(0);
      motor4.action();
      //motor4.reboot();
      delay(50);
      break;
    case 2:
      motor2.goalVel(-vel);
      motor2.action();
      break;
    case 3:
      motor2.goalVel(vel);
      motor2.action();
      break;
    case 4:
      motor3.goalVel(-vel);
      motor3.action();
      break;
    case 5:
      motor3.goalVel(vel);
      motor3.action();
      break;
    case 6:
      motor1.goalVel(-vel);
      motor1.action();
      break;
    case 7:
      motor1.goalVel(vel);
      motor1.action();
      break;
    case 8:
      openGrippers();
      break;
    case 9:
      closeGrippers();
      break;
    default:
      break;
  }/*
  if (incomingByte == 2){
     ekstra = -4;
      if (x == 0)
      {
          motor2.getPos();
          int theta = readPos()-90;
          motor3.getPos();
          int theta2 = readPos()-90;
          x = xPos(theta, theta2);
      }
      motor2.goalVel(-6);
      motor2.action();
      y = 0;
      moveUp();
  }
  else if(incomingByte == 3){
      ekstra = 4;
      if (x == 0)
      {
          motor2.getPos();
          int theta = readPos()-90;
          motor3.getPos();
          int theta2 = readPos()-90;
          x = xPos(theta, theta2);
      }
      y = 0;
      //moveUp();
      motor2.goalVel(6);
      motor2.action();
    }
    else if (incomingByte == 4){
      motor3.goalVel(6);
      motor3.action();
    }
     else if (incomingByte == 5){
      motor3.goalVel(-6);
      motor3.action();
    }
    

  //delay(10);*/

}
