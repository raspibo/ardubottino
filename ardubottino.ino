#include <IRremoteInt.h>
#include <IRremote.h>

 int IRpin =2; //pin del sensore IR
 IRrecv irrecv(IRpin);
 long duration;
 long dursx;
 long durdx;

 const int en1 = 10; //pin enable
 const int en2 = 5;
 const int m1fd = 4; //controlli motore
 const int m2fd = 3;
 const int m1bk = 8;
 const int m2bk = 11;
 const int pinping = 7; //pin sensore distanza
 const int sterzo = 600; //durata dello sterzo
 const int ridster = 550; //diminuzione per i cingoli
 const int distanza = 500; //distanza minima ostacoli
 int motore1 = 78;//pwm motore dx
 int motore2 = 72;//pwm motore sx
 boolean automat = false;

 decode_results results;

 void setup() {
  Serial.begin(9600);
  pinMode(m1fd, OUTPUT);
  pinMode(m1bk, OUTPUT);
  pinMode(m2fd, OUTPUT);
  pinMode(m2bk, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  analogWrite(en1,motore1);
  analogWrite(en2,motore2);
  irrecv.enableIRIn();
 }

 void loop(){
   if(irrecv.decode(&results)){
     if (!automat) {
       if(results.value == 2742014623 || results.value == 31889539){
         dritto();
       } else if (results.value == 2383694249 || results.value == 2039764837 ) {
         sinistra();
       } else if (results.value == 15111918 || results.value == 2725237002) {
         rovescia();
       } else if (results.value == 1665824360 || results.value == 3250666572 ) {
         destra();
       } else if (results.value == 1463772700|| results.value == 4173897784) {
         motore1 += 1;
         debugmot(motore1,motore2);
         analogWrite(en1,motore1);
       } else if (results.value == 2117945733|| results.value == 2461875145) {
         motore2 += 1;
         debugmot(motore1,motore2);
         analogWrite(en2,motore2);
       } else if (results.value == 188078261|| results.value == 532007673) {
         motore1 -=1;
         debugmot(motore1,motore2);
         analogWrite(en1,motore1);
       } else if (results.value == 2411542288|| results.value == 2067612876) {
         motore2 -=1;
         debugmot(motore1,motore2);
         analogWrite(en2,motore2);
       } else  {
         ferma();
       }
     }
     if (results.value == 3969632309 || results.value == 18594425) {
       if (automat) {
         automat = false;
         ferma();
         delayMicroseconds(300000);
       }else{
         automat = true;
         dritto();
         delayMicroseconds(300000);
       }
     }
     irrecv.resume();
   }
   if (automat && echo() < distanza){ //trovato un ostacolo
    analogWrite(en1,170);
    analogWrite(en2,170);
    ferma();
    sinistra();
    delay(sterzo);
    ferma();
    dursx = echo(); //guarda a sinistra
    destra();
    delay(sterzo*2);
    ferma();
    durdx = echo(); //guarda a destra
    if (dursx < distanza && durdx < distanza){
      destra();
      delay(sterzo*2);
    } else if (dursx > durdx) {
      sinistra();
      Serial.println(1);
      delay(sterzo+ridster);
    } else {
      sinistra();
      delay(ridster);
    }
    //sceglie dove c'è più spazio
    dritto();
    analogWrite(en1,motore1);
    analogWrite(en2,motore2);
  }
 }


 void dritto() {
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,LOW);
  digitalWrite(m2fd,HIGH);
  digitalWrite(m1fd,HIGH);
 }

 void rovescia() {
  digitalWrite(m1fd,LOW);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,HIGH);
  digitalWrite(m2bk,HIGH);
 }

 void destra() {
  digitalWrite(m1fd,LOW);
  digitalWrite(m2fd,HIGH);
  digitalWrite(m1bk,HIGH);
  digitalWrite(m2bk,LOW);
 }

 void sinistra() {
  digitalWrite(m1fd,HIGH);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,HIGH);
 }

 void debugmot(int sm1, int sm2) {
  Serial.print(sm1);
  Serial.print(",");
  Serial.println(sm2);
 }

 void ferma() {
  digitalWrite(m1fd,LOW);
  digitalWrite(m2fd,LOW);
  digitalWrite(m1bk,LOW);
  digitalWrite(m2bk,LOW);
  delay(50);
 }

 long echo() {
  pinMode(pinping, OUTPUT);
  digitalWrite(pinping, LOW);
  delayMicroseconds(2);
  digitalWrite(pinping, HIGH);
  delayMicroseconds(5);
  digitalWrite(pinping, LOW);
  pinMode(pinping, INPUT);
  duration = pulseIn(pinping,HIGH);
  return duration;
 }
