#include <Arduino.h>

# define KP 2
# define KI 0
# define KD 0.05

int pin1_L=12;
int pin2_L=11;
int pin3_L=8;
int pin4_L=10;

int pin1_R=7;
int pin2_R=9;
int pin3_R=5;
int pin4_R=6;

int C_Value;
int R_Value;
int L_Value;
int lastDeff;

float Duty = 0;
//float lastDuty;
//float LPFk = 0.05;
float dt, preTime;
float vol;
float Deff_RL, Deff_RL_I, Deff_RL_D, preDeff_RL;


inline void RUN_RIGHT(int power){
  if (power>0){
    digitalWrite(pin1_R , 1);
    analogWrite(pin2_R , power);
    digitalWrite(pin3_R , 0);
    digitalWrite(pin4_R , 0);
  }
    else{
    power = abs(power);
    if(power>255){
      power=255;
    }
    digitalWrite(pin1_R , 0);
    digitalWrite(pin2_R , 0);
    digitalWrite(pin3_R , 1);
    analogWrite(pin4_R , power);
    }
  }


inline void RUN_LEFT(int power){
  if (power>0){
    digitalWrite(pin1_L , 1);
    analogWrite(pin2_L , power);
    digitalWrite(pin3_L , 0);
    digitalWrite(pin4_L , 0);
  }
    else{
    power = abs(power);
    if(power>255){
      power=255;
    }
    digitalWrite(pin1_L , 0);
    digitalWrite(pin2_L , 0);
    digitalWrite(pin3_L , 1);
    analogWrite(pin4_L , power);
    }
  }

inline void BRAKE(){
    digitalWrite(pin1_L , 1);
    digitalWrite(pin2_L , 0);
    digitalWrite(pin3_L , 1);
    digitalWrite(pin4_L , 0);

    digitalWrite(pin1_R , 1);
    digitalWrite(pin2_R , 0);
    digitalWrite(pin3_R , 1);
    digitalWrite(pin4_R , 0);
}


inline void RUN(int Deff){
  if (Deff>0){
    RUN_LEFT(255);
    RUN_RIGHT(255 - Deff);
  }
  else{
    Deff = abs(Deff);
    RUN_LEFT(255 - Deff);
    RUN_RIGHT(255);
  }
}


void setup() {
  pinMode(pin1_L, OUTPUT); 
  pinMode(pin2_L, OUTPUT); 
  pinMode(pin3_L, OUTPUT); 
  pinMode(pin4_L, OUTPUT); 

  pinMode(pin1_R, OUTPUT); 
  pinMode(pin2_R, OUTPUT); 
  pinMode(pin3_R, OUTPUT); 
  pinMode(pin4_R, OUTPUT);  

 Serial.begin(9600);

 preTime = micros();
}

void loop() {
  dt =(micros()-preTime)/1000000;
  preTime=micros();
  
  C_Value = analogRead(6);
  R_Value = analogRead(7);
  L_Value = analogRead(5);
  
  Deff_RL = (R_Value - L_Value);
 
//  Deff_RL = round(Deff_RLf);
 
  if(abs(Deff_RL)>50){
    lastDeff=Deff_RL;
  }

  Deff_RL_I += Deff_RL*dt;
  Deff_RL_D = (Deff_RL-preDeff_RL)/dt;
  preDeff_RL = Deff_RL;

  Duty = KP*Deff_RL + KI * Deff_RL_I + KD * Deff_RL_D;

  //Duty += LPFk*((KP*Deff_RL + KI * Deff_RL_I + KD * Deff_RL_D)-lastDuty);
  //lastDuty = Duty;
 
  Serial.println(Duty);

/*  Serial.print(C_Value);
  Serial.print(",");
  Serial.print(R_Value);
  Serial.print(",");
  Serial.println(L_Value);
*/

  if (C_Value>100){
    if(abs(Duty)<40){
      RUN(0);
    }
    else{
    RUN(Duty);
    }
  }
  else{
    if(lastDeff>0){
      RUN(400);
    }
    else{
      RUN(-400);
    }
  }
}
