#include <Wire.h>

#define MPU 0x68

int16_t ax=10;
int16_t ay=20;
int16_t az=30;
int16_t gx=0;
int16_t gy=0;
int16_t gz=0;
int16_t temp=25;

int16_t dt=0;
int16_t tiempo_prev=0;

float angulo=0;
float anguloant=0;



#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

TaskHandle_t Task1; 
TaskHandle_t Task2; 


float sinled[5];
float conled[5];
int val[5];

int velocidad=100;

int vgiro;
int vel;

int deriva=0;

float filtrojames1=0.96;
float filtrojames2=1.0-filtrojames1;


boolean muevemotores=true;

int umbral=970;
int umbral2=1200;

void wdt(){
    esp_task_wdt_init(30,false);
    esp_task_wdt_reset();
    yield(); 
}

void envia(){

   String dato="";
   for(int i=0;i<5;i++){          
              dato+=val[i];
              if(i!=4){ dato+=","; }
   }
      
   //SerialBT.println(dato);

   int v=val[1]-val[3];

   //SerialBT.println(v);

   SerialBT.println(angulo);
  
}

void espera(int milisegundos){
  
  int tiempo=millis();
  
  envia();
  
  while((millis()-tiempo)<milisegundos){
                 
      delay(5);

      envia();
      
      wdt();  
  }
  
}


void updateGiro()
{
   dt = millis() - tiempo_prev;
   tiempo_prev = millis();
   angulo = (((float)gz) / 131.0)*((float)dt) / 1000.0;
   if((angulo>-0.02)&&(angulo<0.02)){ angulo=0; }
   angulo+=anguloant;
   
   //angulo += 0.000724*((float)dt);  //sirve para corregir la deriva
   
   anguloant = angulo;

   
}



void leempu6050() {

  Wire.beginTransmission(MPU);
  Wire.write(0x47); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,2, true);

  int tiempo=millis();  
  while(Wire.available() < 2){
      if((millis()-tiempo)>50){ break; }
  }
  
  gz=Wire.read()<<8|Wire.read();

  
  
    
}


void girocompleto(){

  anguloant=0;
  
  while(angulo<160){                            
          ledcWrite(0,0);ledcWrite(1,vel);
          ledcWrite(2,vel);ledcWrite(3,0); 
          espera(1);              
  }

  ledcWrite(0,0);ledcWrite(1,vel);
  ledcWrite(2,0);ledcWrite(3,vel);
   
  return;

  if(vgiro>0){
              
      while(angulo<160){                            
          ledcWrite(0,0);ledcWrite(1,vel);
          ledcWrite(2,vel);ledcWrite(3,0); 
          espera(10);              
      }
              
  }else{

      while(angulo>-160){                             
          ledcWrite(0,vel);ledcWrite(1,0);
          ledcWrite(2,0);ledcWrite(3,vel); 
          espera(10);              
      }             
              
  }
    
}

void setup(){

   Wire.begin(21,22);
  
   Wire.beginTransmission(MPU);
   Wire.write(0x6B);
   Wire.write(0);
   Wire.endTransmission(true);
  
   Wire.beginTransmission(MPU);
   Wire.write(0x19);   //Sample Rate
   Wire.write(0x00);   //  8000 / 1 + 0
   Wire.endTransmission(true);
  
   Wire.beginTransmission(MPU);
   Wire.write(0x1C);   //Setting Accel
   Wire.write(0x18);      //  + / - 16g
   Wire.endTransmission(true);

   
  Serial.begin(115200);

  ledcAttachPin(12,0);ledcSetup(0,100,8);
  ledcAttachPin(13,1);ledcSetup(1,100,8);
  ledcAttachPin(14,2);ledcSetup(2,100,8);
  ledcAttachPin(27,3);ledcSetup(3,100,8);
  
  
  SerialBT.begin("BOTMAZE"); //Bluetooth device name
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

//  pinMode(12,OUTPUT);pinMode(13,OUTPUT);
//  pinMode(14,OUTPUT);pinMode(27,OUTPUT);

  ledcWrite(0,0);ledcWrite(1,0);
  ledcWrite(2,0);ledcWrite(3,0);
  
   
  pinMode(15,OUTPUT);
  
  pinMode(26,INPUT);
  pinMode(25,INPUT);
  pinMode(33,INPUT);
  pinMode(32,INPUT);
  pinMode(35,INPUT);

  
  for(int i=0;i<5;i++){
      conled[i]=0.0;
      sinled[i]=0.0;      
  }

  

   xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 

   espera(5000);
   
   anguloant=0;  
   
}



void Task1code( void * pvParameters ){    // Lectura sensore IR

  
  for(;;){  
     
     digitalWrite(15,LOW);delay(1);
  
     int valor=analogRead(26);
     sinled[0]=(sinled[0]*filtrojames1)+(((float)valor)*filtrojames2);

     valor=analogRead(25);
     sinled[1]=(sinled[1]*filtrojames1)+(((float)valor)*filtrojames2);

     valor=analogRead(33);
     sinled[2]=(sinled[2]*filtrojames1)+(((float)valor)*filtrojames2);

     valor=analogRead(32);
     sinled[3]=(sinled[3]*filtrojames1)+(((float)valor)*filtrojames2);

     valor=analogRead(35);
     sinled[4]=(sinled[4]*filtrojames1)+(((float)valor)*filtrojames2); 


     digitalWrite(15,HIGH);delay(1);
  
     valor=analogRead(26);
     conled[0]=(conled[0]*filtrojames1)+(((float)valor)*filtrojames2);

     valor=analogRead(25);
     conled[1]=(conled[1]*filtrojames1)+(((float)valor)*filtrojames2);

     valor=analogRead(33);
     conled[2]=(conled[2]*filtrojames1)+(((float)valor)*filtrojames2);

     valor=analogRead(32);
     conled[3]=(conled[3]*filtrojames1)+(((float)valor)*filtrojames2);

     valor=analogRead(35);
     conled[4]=(conled[4]*filtrojames1)+(((float)valor)*filtrojames2);

     for(int i=0;i<5;i++){
          val[i]=conled[i]-sinled[i];          
     }

    leempu6050();

    //Hacia la izquierda el angulo aumenta, hacia la derecha el angulo disminuye.
    updateGiro();

    
    wdt();    
    
  } 
  
}

void loop(){

  if(muevemotores){
  
    //int val=(int)(conled[2]-sinled[2]); 
  
    vel=(255*velocidad)/100;
        
    int v=(val[1]-val[3])+(deriva/100);
  
    vgiro=val[0]+val[1]-val[3]-val[4];
  
    if(val[2]>umbral){
  
        anguloant=0;
        
        if(vgiro>0){
                
            while(angulo<75){          

                if((val[0]>umbral)&&(val[1]>umbral)&&(val[2]>umbral2)&&(val[3]>umbral)&&(val[4]>umbral)){  girocompleto();  break; }      
                                                  
                ledcWrite(0,0);ledcWrite(1,vel);
                ledcWrite(2,0);ledcWrite(3,0); 
                espera(1);                       
                
            }
                           
        }else{
  
            while(angulo>-75){      
                
                if((val[0]>umbral)&&(val[1]>umbral)&&(val[2]>umbral2)&&(val[3]>umbral)&&(val[4]>umbral)){  girocompleto();  break; }      
                                       
                ledcWrite(0,0);ledcWrite(1,0);
                ledcWrite(2,0);ledcWrite(3,vel); 
                espera(1);   
                                    
            }           

              
                
        }

        ledcWrite(0,0);ledcWrite(1,vel);
        ledcWrite(2,0);ledcWrite(3,vel);
  
        deriva=0;
        
    }else {
    
        if(v>0){
  
            ledcWrite(0,0);ledcWrite(1,vel);      
  
                
            if(v<4000){
                int v2=map(v,0,4000,255,0);
                int vel2=(v2*velocidad)/100;                  
                ledcWrite(2,0);ledcWrite(3,vel2);    
  
                deriva+=vel2;
                            
            }else{
                int v2=map(v,2400,4000,0,255);
                int vel2=(v2*velocidad)/100;
                ledcWrite(2,0);ledcWrite(3,0); 
  
                deriva+=vel;
            }
            
        }else{
  
            ledcWrite(2,0);ledcWrite(3,vel);    
                       
            if(v>-4000){
                  
                int v2=map(v,-4000,0,0,255);
                int vel2=(v2*velocidad)/100;
                ledcWrite(0,0);ledcWrite(1,vel2);   
  
                deriva-=vel2;
              
            }else{
              
                int v2=map(v,-4000,-2400,255,0);
                int vel2=(v2*velocidad)/100;           
                ledcWrite(0,vel);ledcWrite(1,0);
  
                deriva-=vel;
            }
            
        }
  
    }

  }

  espera(100); 
  
}
