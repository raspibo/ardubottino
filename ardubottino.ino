/*Ardubottino
 * Basic control for the Ardubottino rover, via IR or auto mode.
 * 
 * This code is in the public domain.
 * 
 * Emiliano Mattioli
 * last modified 14 August 2015
 */

#include <IRremote.h>
#include "ardubottino.h"

//PINS
IRrecv irrecv(9); //IR sensor output to pin       #9
const int en1 = 10; //Motor 1 enable to pin       #10
const int m1fd = 8; //Motor 1 fd control to pin   #4
const int m1bk = 4; //Motor 1 bk control to pin   #8
const int en2 = 5;  //Motor 2 enable to pin       #5
const int m2fd = 6; //Motor 2 fd control to pin   #6
const int m2bk = 11;//Motor 2 bk control to pin   #11
const int ping = 7; //Echo sensor to pin          #7
const int fall = 12;//Fall sensor pin             #12
const int led = 13; //Onboard led                 #13

//IR
decode_results results;
unsigned long curr_result;
unsigned long prev_result;

//Echo
int distlt;
int distrt;
const int tooclose = 500;

//Falling
bool falling = false;

//Auto
const int steer = 800;
const int steeravoid = 100;
bool automat = false;
bool program = false;

//Speed
int motor1PWM = 72;
int motor2PWM = 80;
bool turbo = false;

//Encoders
volatile long right_interval = 0;
volatile long left_interval = 0;
volatile int right_count = 0;
volatile int left_count = 0;
volatile long next_check_right = 0;
volatile long next_check_left = 0;
bool motion = false;
int stall = 0;

//Program Mode
unsigned int moves[50][2];
unsigned int pointer = 0;

//Debug
bool debug = false;

void setup() {
  pinMode(m1fd,OUTPUT);
  pinMode(m1bk,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(m2fd,OUTPUT);
  pinMode(m2bk,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(fall,INPUT);
  pinMode(led,OUTPUT);
  analogWrite(en1,motor1PWM);
  analogWrite(en2,motor2PWM);
  attachInterrupt(0, right_encoder,RISING);
  attachInterrupt(1, left_encoder,RISING);
  irrecv.enableIRIn();
  delay(800);
  if (debug) {
    Serial.begin(9600);
    Serial.println("READY");
  }
}

void loop() {
   if (debug) {        
    Serial.print(" L Encoder:");
    Serial.print(left_count);    
    Serial.print(" ; ");   
    Serial.print(left_interval);
    Serial.print("           ");
    Serial.print("R Encoder:");
    Serial.print(right_count);    
    Serial.print(" ; ");   
    Serial.print(right_interval);
    Serial.print(" Distance:");
    Serial.print(echo());
    Serial.print(" Motor1:");
    Serial.print(motor1PWM);
    Serial.print(" Motor2:");
    Serial.print(motor2PWM);
    Serial.print("  IR:");
    Serial.print(curr_result);
    Serial.print("\n");
  }
  if (digitalRead(fall) && !automat && !program) {
    falling = true;

    noMotion();
  } else {
    falling = false;
  }  
  if (irrecv.decode(&results)) {
    curr_result = results.value;
    if (curr_result == prev_result) {
      curr_result = 0;
    } else {
      prev_result = curr_result;
    }
    if (!automat && !program) {//manual mode
      if(curr_result == FORWARDA || curr_result == FORWARDB){
         if(!falling) {
          forward();
          blinkOK();
         }
      } else if (curr_result == LEFTA || curr_result == LEFTB) {
         left();
         blinkOK();
      } else if (curr_result == BACKA || curr_result == BACKB) {
         backward();
         if(falling) {
          while(digitalRead(fall)) {}
          falling = false;
         }
         blinkOK();
      } else if (curr_result == RIGHTA || curr_result == RIGHTB) {
         right();
         blinkOK();
      } else if (curr_result == VOLUMEUPA|| curr_result == VOLUMEUPB) {
         motor2PWM += 1;                  
         motor2PWM = PWM_MinMaxCheck(motor2PWM);
         analogWrite(en2,motor2PWM);
         blinkOK();
      } else if (curr_result == PROGRAMUPA|| curr_result == PROGRAMUPB) {
         motor1PWM += 1;
         motor1PWM = PWM_MinMaxCheck(motor1PWM);
         analogWrite(en1,motor1PWM);
         blinkOK();
      } else if (curr_result == VOLUMEDOWNA|| curr_result == VOLUMEDOWNB) {
         motor2PWM -= 1;
         motor2PWM = PWM_MinMaxCheck(motor2PWM);
         analogWrite(en2,motor2PWM);
         blinkOK();
       } else if (curr_result == PROGRAMDOWNA|| curr_result == PROGRAMDOWNB) {
         motor1PWM -= 1;
         motor1PWM = PWM_MinMaxCheck(motor1PWM);
         analogWrite(en1,motor1PWM);
         blinkOK();
       } 
    }
    if (program) {
      if (curr_result == OKA || curr_result == OKB) {
        blinkOK();
        executeProgram();
        clearMoves();
        noMotion();
      } else if (curr_result == NUM1A || curr_result == NUM1B) {
        moves[pointer][0] = (moves[pointer][0]*10) +1;
        blinkOK();
      } else if (curr_result == NUM2A || curr_result == NUM2A) {
        moves[pointer][0] = (moves[pointer][0]*10) +2;
        blinkOK();
      } else if (curr_result == NUM3A || curr_result == NUM3B) {
        moves[pointer][0] = (moves[pointer][0]*10) +3;
        blinkOK();
      } else if (curr_result == NUM4A || curr_result == NUM4B) {
        moves[pointer][0] = (moves[pointer][0]*10) +4;
        blinkOK();
      } else if (curr_result == NUM5A || curr_result == NUM5B) {
        moves[pointer][0] = (moves[pointer][0]*10) +5;
        blinkOK();
      } else if (curr_result == NUM6A || curr_result == NUM6B) {
        moves[pointer][0] = (moves[pointer][0]*10) +6;
        blinkOK();
      } else if (curr_result == NUM7A || curr_result == NUM7B) {
        moves[pointer][0] = (moves[pointer][0]*10) +7;
        blinkOK();
      } else if (curr_result == NUM8A || curr_result == NUM8B) {
        moves[pointer][0] = (moves[pointer][0]*10) +8;
        blinkOK();
      } else if (curr_result == NUM9A || curr_result == NUM9B) {
        moves[pointer][0] = (moves[pointer][0]*10) +9;
        blinkOK();
      } else if (curr_result == NUM0A || curr_result == NUM0B) {
        moves[pointer][0] = (moves[pointer][0]*10);   
        blinkOK();
      } else if (curr_result == FORWARDA || curr_result == FORWARDB) {
        moves[pointer][1] = 1;
        pointer ++;
        blinkOK();
      } else if (curr_result == BACKA || curr_result == BACKB) {
        moves[pointer][1] = 2;
        pointer ++;
        blinkOK();
      } else if (curr_result == LEFTA || curr_result == LEFTB) {
        moves[pointer][1] = 3;
        pointer ++;
        blinkOK();
      } else if (curr_result == RIGHTA || curr_result == RIGHTB) {
        moves[pointer][1] = 4;
        pointer ++;
        blinkOK();
      } else if (curr_result == WINA || curr_result == WINB) {
        clearMoves();
        blinkOK();
      } else {
        blinkNO();
      }
      if(moves[pointer][0] > 99) {
       moves[pointer][0] = moves[pointer][0]-((moves[pointer][0]/100)*100);
      }
      if (pointer >49) {
        executeProgram();
      }
    }
    if (curr_result == WINA|| curr_result == WINB) { //stop, works also as an emergency stop
      noMotion();
      automat = false;
      blinkOK();
    }
    if (curr_result == REDA|| curr_result == REDB) { //going auto
      blinkOK();
      automat = true;
      program = false;
      noMotion();
      delay(500);
      forward();
    }
    if (curr_result == GREENA|| curr_result == GREENB) { //going manual
      blinkOK();
      automat = false;
      program = false;
      noMotion();
      delay(500);
    }
    if (curr_result == YELLOWA|| curr_result == YELLOWB) { //going programmed
      blinkOK();
      automat = false;
      program = true;
      noMotion();
      clearMoves();
      delay(500);
    }
    if (curr_result == BLUEA|| curr_result == BLUEB) { //calibration
      blinkOK();
      program = false;
      automat = false;
      noMotion();
      delay(500);
      forward();
      int loops = 0;
      int last_left = left_count;
      int last_right = right_count;
      bool left_pass = false;
      bool right_pass = false;
      while (loops < 12) {
        if (last_left != left_count) {
          left_pass = true;
        }
        if (last_right != right_count) {
          right_pass = true;
        }
        if (left_pass && right_pass) {
          left_pass = false;
          right_pass = false;
          last_left = left_count;
          last_right = right_count;
          if (abs(right_interval - left_interval) < 80) {
            loops +=1;
          } else if (right_interval > left_interval) {
            motor1PWM += 1;
            analogWrite(en1,motor1PWM);
            loops = 0;
          } else {
            motor1PWM -= 1;
            analogWrite(en1,motor1PWM);
            loops = 0;
          }
          if(debug) {
            Serial.print(motor1PWM);
            Serial.print(" ");
            Serial.println(motor2PWM);
          }
        }
      }
      noMotion();
    }
    if (curr_result == MUTEA|| curr_result == MUTEB) { //turbo on/off
       blinkOK();
       if (turbo) {
           turbo = false;
           motor1PWM -= 100;
           motor2PWM -= 100;
         } else {
           turbo = true;
           motor1PWM += 100;
           motor2PWM += 100;
         }
         motor1PWM = PWM_MinMaxCheck(motor1PWM);
         motor2PWM = PWM_MinMaxCheck(motor2PWM);
         analogWrite(en1,motor1PWM);
         analogWrite(en2,motor2PWM);
         delay(500);
    }
    irrecv.resume();
  }
  if (automat) {
    if (digitalRead(fall)) {
      backward();
      delay(2000);
      left();
      delay(steer*2);
      forward();
    }
  }
  if (automat && echo() < tooclose) { //automatic mode
    analogWrite(en1,PWM_MinMaxCheck(motor1PWM +100));
    analogWrite(en2,PWM_MinMaxCheck(motor2PWM +100));
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
  if(motion && automat) { //check if any of the wheels is stuck
    if((next_check_right < millis()) || (next_check_left < millis())) {
      stall +=1;
      if (stall > 15) {        
        backward();
        delay(500);
        right();
        delay(steer);
        forward();
        stall = 0;
      }
    } else {
      stall = 0;
    }
  }
}

int PWM_MinMaxCheck(int pwm) {
  if(pwm >255) {
    pwm =255;
  }
  if(pwm <0) {
    pwm = 0;
  }
  return pwm;
}


void executeProgram() {
  int right_count_program = right_count;
  int left_count_program = left_count;
  for (int x=0;x<=49;x++){
    if (moves[x][1] == 0) {
      break;
    } else if (moves[x][1] == 1){
      forward();
    } else if (moves[x][1] == 2){
      backward();
    } else if (moves[x][1] == 3){
      left();
    } else if (moves[x][1] == 4){
      right();
    }
    while(moves[x][0] > 0) {
      if (right_count_program != right_count && left_count_program != left_count) {
        moves[x][0] --;
        right_count_program = right_count;
        left_count_program = left_count;
      }
      if (digitalRead(fall)) {      
        noMotion();
        program = false;
        return;
      }
    }
    moves[x][1] = 0;
  }
  program = false;
  noMotion();
}

void noMotion() {
  stall = 0;
  motion = false;
  digitalWrite(m1fd,LOW);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,LOW);
  delay(50);
}

void forward() {
  motion = true;
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,LOW);
  digitalWrite(m2fd,HIGH);
  digitalWrite(m1fd,HIGH);
}

void backward() {
  motion = true;
  digitalWrite(m1fd,LOW);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,HIGH);
  digitalWrite(m2bk,HIGH);
}

void right() {
  motion = true;
  digitalWrite(m1fd,HIGH);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,HIGH);
}

void left() {
  motion = true;
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

void clearMoves() {
  pointer = 0;
  for (int x = 0; x<=49; x++) {
    moves[x][0] = 0;
    moves[x][1] = 0;
  }
}

void blinkOK() {
  for(int x = 0; x <=3; x++){
    digitalWrite(led,HIGH);
    delay(50);
    digitalWrite(led,LOW);
    delay(50);
  }
}

void blinkNO() {
  for(int x = 0; x <=2; x++) {
    digitalWrite(led,HIGH);
    delay(200);
    digitalWrite(led,LOW);
  }
}

void right_encoder() {
  static unsigned long last_interrupt = 0;
  unsigned long time_interrupt = millis();
  if(time_interrupt - last_interrupt >70){
    right_interval = time_interrupt - last_interrupt;
    next_check_right = millis()+right_interval*2;
    right_count +=1;
    last_interrupt = time_interrupt;
  }
}

void left_encoder() {
  static unsigned long last_interrupt = 0;
  unsigned long time_interrupt = millis();
  if(time_interrupt - last_interrupt >70) {
    left_interval = millis() - last_interrupt;
    next_check_left = millis()+left_interval*2;
    left_count +=1;
    last_interrupt = millis();
  }
}

