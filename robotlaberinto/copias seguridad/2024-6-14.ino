#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

TaskHandle_t Task1;  // declaramos para poder ejecutar en otro core


float sinled[5];
float conled[5];
int val[5];

int velocidad=100;

int deriva=0;

void wdt(){
    esp_task_wdt_init(30,false);
    esp_task_wdt_reset();
    yield(); 
}

void espera(int milisegundos){
  int tiempo=millis();

  while((millis()-tiempo)<milisegundos){
      String dato="";
      for(int i=0;i<5;i++){          
              dato+=val[i];
              if(i!=4){ dato+=","; }
      }
      
      //SerialBT.println(dato);

      int v=val[1]-val[3];

      SerialBT.println(v);
  
      delay(10);
  }
  
}

void setup(){

  Serial.begin(115200);

  ledcAttachPin(12,0);ledcSetup(0,200,8);
  ledcAttachPin(13,1);ledcSetup(1,200,8);
  ledcAttachPin(14,2);ledcSetup(2,200,8);
  ledcAttachPin(27,3);ledcSetup(3,200,8);
  
  
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
                    
}


void Task1code( void * pvParameters ){    // en este Core recogemos las peticiones web

  
  for(;;){  
     
     digitalWrite(15,LOW);delay(1);
  
     int valor=analogRead(26);
     sinled[0]=(sinled[0]*0.99)+(((float)valor)*0.01);

     valor=analogRead(25);
     sinled[1]=(sinled[1]*0.99)+(((float)valor)*0.01);

     valor=analogRead(33);
     sinled[2]=(sinled[2]*0.99)+(((float)valor)*0.01);

     valor=analogRead(32);
     sinled[3]=(sinled[3]*0.99)+(((float)valor)*0.01);

     valor=analogRead(35);
     sinled[4]=(sinled[4]*0.99)+(((float)valor)*0.01); 


     digitalWrite(15,HIGH);delay(1);
  
     valor=analogRead(26);
     conled[0]=(conled[0]*0.99)+(((float)valor)*0.01);

     valor=analogRead(25);
     conled[1]=(conled[1]*0.99)+(((float)valor)*0.01);

     valor=analogRead(33);
     conled[2]=(conled[2]*0.99)+(((float)valor)*0.01);

     valor=analogRead(32);
     conled[3]=(conled[3]*0.99)+(((float)valor)*0.01);

     valor=analogRead(35);
     conled[4]=(conled[4]*0.99)+(((float)valor)*0.01);

     for(int i=0;i<5;i++){
          val[i]=conled[i]-sinled[i];          
     }

    wdt();    
    
  } 
  
}

void loop(){

  //int val=(int)(conled[2]-sinled[2]); 

  int vel=(255*velocidad)/100;
      
  int v=(val[1]-val[3])+(deriva/100);

  int vgiro=val[0]+val[1]-val[3]-val[4];

  if(val[2]>1000){

      if(vgiro>0){
        
          ledcWrite(0,0);ledcWrite(1,vel);
          //ledcWrite(2,vel);ledcWrite(3,0); 
          ledcWrite(2,0);ledcWrite(3,0); 
          
          deriva+=vel;
          deriva=0;
        
      }else{

           ledcWrite(0,vel);ledcWrite(1,0);
           //ledcWrite(2,0);ledcWrite(3,vel);        
           ledcWrite(2,0);ledcWrite(3,0);
           
          deriva-=vel;
          deriva=0;
      }
    
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
              ledcWrite(2,vel);ledcWrite(3,0); 

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
  /*    
  if(val[0]>3000){ 
     ledcWrite(1,40);    
  }else{
     ledcWrite(1,0);    
     contador++;
  }

  if(val[4]>3000){ 
     digitalWrite(27,HIGH);       
  }else{
     digitalWrite(27,LOW);        
     contador++;
  }

  if(contador==2){
    digitalWrite(13,HIGH);       
    digitalWrite(27,HIGH);     
  }

  if(val[1]>3000){
    digitalWrite(12,LOW);digitalWrite(13,HIGH);  
    digitalWrite(14,HIGH);digitalWrite(27,LOW);    
  }else if(val[3]>3000){
    digitalWrite(12,HIGH);digitalWrite(13,LOW);  
    digitalWrite(14,LOW);digitalWrite(27,HIGH);    
  }else if(val[2]>3000){
    digitalWrite(12,LOW);digitalWrite(13,HIGH);  
    digitalWrite(14,HIGH);digitalWrite(27,LOW);    
  }
    
  espera(100); 
  
 */
  
  //Serial.println(val);

  espera(100); 
  
}
