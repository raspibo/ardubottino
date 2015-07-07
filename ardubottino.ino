/*Ardubottino
 * Basic control for the Ardubottino rover, via IR or auto mode.
 * 
 * This code is in the public domain.
 * 
 * Emiliano Mattioli
 * last modified 5 July 2015
 */

#include <IRremote.h>


//PINS
IRrecv irrecv(2); //IR sensor output to pin       #2
const int en1 = 10; //Motor 1 enable to pin       #10
const int m1fd = 8; //Motor 1 fd control to pin   #4
const int m1bk = 4; //Motor 1 bk control to pin   #8
const int en2 = 5;  //Motor 2 enable to pin       #5
const int m2fd = 3; //Motor 2 fd control to pin   #3
const int m2bk = 11;//Motor 2 bk control to pin   #11
const int ping = 7; //Echo sensor to pin          #7


//IR
decode_results results;

//Echo

int distlt;
int distrt;
const int tooclose = 500;

//Auto
const int steer = 800;
const int steeravoid = 100;
bool automat = false;

//Speed
int motor1PWM = 75;
int motor2PWM = 80;
bool turbo = false;

//Debug
bool debug = false;

void setup() {
  pinMode(m1fd,OUTPUT);
  pinMode(m1bk,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(m2fd,OUTPUT);
  pinMode(m2bk,OUTPUT);
  pinMode(en2,OUTPUT);
  analogWrite(en1,motor1PWM);
  analogWrite(en2,motor2PWM);

  irrecv.enableIRIn();
  delay(800);
  if (debug) {
    Serial.begin(9600);
    Serial.println("READY");
  }
}

void loop() {
   if (debug) {
      Serial.print("IR:");
      Serial.print(results.value);
      Serial.print(" Distance:");
      Serial.print(echo());
      Serial.print(" Motor1:");
      Serial.print(motor1PWM);
      Serial.print(" Motor2:");
      Serial.println(motor2PWM);
    }
  if (irrecv.decode(&results)) {
   
    if (!automat) {
      if(results.value == 2742014623 || results.value == 31889539){
         forward();
      } else if (results.value == 2383694249 || results.value == 2039764837 ) {
         left();
      } else if (results.value == 15111918 || results.value == 2725237002) {
         backward();
      } else if (results.value == 1665824360 || results.value == 3250666572 ) {
         right();
      } else if (results.value == 1463772700|| results.value == 4173897784) {
         motor2PWM += 1;
         analogWrite(en2,motor2PWM);
      } else if (results.value == 2117945733|| results.value == 2461875145) {
         motor1PWM += 1;
         analogWrite(en1,motor1PWM);
      } else if (results.value == 188078261|| results.value == 532007673) {
         motor2PWM -= 1;
         analogWrite(en2,motor2PWM);
       } else if (results.value == 2411542288|| results.value == 2067612876) {
         motor1PWM -= 1;
         analogWrite(en1,motor1PWM);
       }  else  {
         noMotion();
       }  
    }
    if (results.value == 3969632309 || results.value == 18594425) {
       if (automat) {
         automat = false;
         noMotion();
         delay(500);
       }else{
         automat = true;
         noMotion();
         delay(500);
         forward();
       }
    }
    if (results.value == 3190304459|| results.value == 480179375) {
       if (turbo) {
           turbo = false;
           motor1PWM -= 100;
           motor2PWM -= 100;
         } else {
           turbo = true;
           motor1PWM += 100;
           motor2PWM += 100;
         }
         analogWrite(en1,motor1PWM);
         analogWrite(en2,motor2PWM);
         delay(500);
    }
    irrecv.resume();
  }
  if (automat && echo() < tooclose) {
    analogWrite(en1,170);
    analogWrite(en2,170);
    right();
    delay(steer);
    distrt = echo();
    left();
    delay(steer*2);
    distlt = echo();
    noMotion();
    if (distrt < tooclose && distlt < tooclose) {
      left();
      delay(steer*2);
    } else if (distrt < distlt) {
      left();
      delay(steeravoid);
    } else {
      right();
      delay(steer*2+steeravoid);
    }
    analogWrite(en1,motor1PWM);
    analogWrite(en2,motor2PWM);
    forward();
  }
}

void noMotion() {
  digitalWrite(m1fd,LOW);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,LOW);
  delay(50);
}

void forward() {
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,LOW);
  digitalWrite(m2fd,HIGH);
  digitalWrite(m1fd,HIGH);
}

void backward() {
  digitalWrite(m1fd,LOW);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,HIGH);
  digitalWrite(m2bk,HIGH);
}

void right() {
  digitalWrite(m1fd,HIGH);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,HIGH);
}

void left() {
  digitalWrite(m1fd,LOW);
  digitalWrite(m2fd,HIGH);
  digitalWrite(m1bk,HIGH);
  digitalWrite(m2bk,LOW);

}

long echo() {
  long distance;
  pinMode(ping, OUTPUT);
  digitalWrite(ping, LOW);
  delayMicroseconds(2);
  digitalWrite(ping, HIGH);
  delayMicroseconds(5);
  digitalWrite(ping, LOW);
  pinMode(ping, INPUT);
  distance = pulseIn(ping, HIGH);
  delay(100);
  return distance;
}

