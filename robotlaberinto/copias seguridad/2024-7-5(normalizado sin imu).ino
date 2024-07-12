#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

TaskHandle_t Task1; 


float sinled[5];
float conled[5];
float val[5];

int velocidad=100;

int vgiro;
int vel;
int vel2;

int deriva=0;

float filtrojames1=0.90;
float filtrojames2=1.0-filtrojames1;

boolean sigue=true;

boolean muevemotores=true;

float umbral=7.6;
float umbral2=6.0;

int calibracion[5][20]={ 
                   {4094,4094,3300,2550,2000,1500,1220,980,840,730,640,575,510,450,415,375,350,320,300,275},
                   {4094,4094,4040,3130,2500,1930,1550,1270,1100,950,825,725,650,575,530,480,440,415,380,365},
                   {4094,4094,3650,2900,2350,1900,1500,1250,1060,910,800,700,625,570,510,460,425,390,360,340},
                   {4094,4000,3240,2400,1850,1470,1150,950,820,690,600,530,470,420,380,350,320,300,275,260},
                   {4094,4000,3200,2470,1880,1550,1250,1050,870,760,670,600,530,490,450,410,385,350,335,310}};


float normaliza(int val,int numsensor){    //Recibe el valor del sensor y lo normaliza para igualar los valores de todos los sensores

   int i=0;
   for(i=0;i<20;i++){
      if(val>=calibracion[numsensor][i]){
          break;      
      }        
   }

   if(i==0){ return 0.0; }
   else if(i==20){ return 20.0; }
   else {

      float nor=(float)map(val,calibracion[numsensor][i-1],calibracion[numsensor][i],0,100);

      float normalizado=((float)(i))+((nor)/100.0);

      return normalizado;
      
   }
     
}


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

   SerialBT.println(val[0]);
  
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



void girocompleto(){

  boolean sigue=true;
  
  vgiro=val[0]+val[1]-val[3]-val[4];

  if(vgiro<0){                       
      
    while(sigue){  
          ledcWrite(0,0);ledcWrite(1,vel);
          ledcWrite(2,vel);ledcWrite(3,0);        
          espera(1);              
          if((val[2]>umbral)&&(val[4]<6.0)){ sigue=false;}
    }
  }else{

    while(sigue){        
          ledcWrite(0,vel);ledcWrite(1,0);
          ledcWrite(2,0);ledcWrite(3,vel);                
          espera(1);              
          if((val[2]>umbral)&&(val[0]<6.0)){ sigue=false;}
    }    
      
  }

  ledcWrite(0,0);ledcWrite(1,vel);
  ledcWrite(2,0);ledcWrite(3,vel);
   
      
}

void setup(){

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

  
   
}



void Task1code( void * pvParameters ){    // Lectura sensore IR

  
  for(;;){  
     
     digitalWrite(15,LOW);delay(2);
  
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


     digitalWrite(15,HIGH);delay(2);
  
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
          int vall=(int)(conled[i]-sinled[i]);             
          val[i]=normaliza(vall,i);                 
     }

     wdt();    
    
  } 
  
}

void loop(){

  if(muevemotores){
  
    //int val=(int)(conled[2]-sinled[2]); 
  
    vel=(255*velocidad)/100;
    vel2=(vel*4)/10;
        
    int v=(val[1]-val[3])+(deriva/100);
  
    vgiro=val[0]+val[1]-val[3]-val[4];
  
    if(val[2]<umbral){

        sigue=true;  
        
        if(vgiro<0){  //Giro a la izquierda

            while(sigue){          

                if((val[0]>2.5)&&(val[1]>3.5)&&(val[2]>umbral)){ sigue=false; }
                
                if((val[0]<umbral2)&&(val[1]<umbral2)&&(val[2]<umbral2)&&(val[3]<umbral2)&&(val[4]<umbral2)){  girocompleto();  break; }      
                                                  
                ledcWrite(0,0);ledcWrite(1,vel);
                ledcWrite(2,0);ledcWrite(3,0); 
                espera(1);                       
                
            }
                           
        }else{   //Giro a la derecha

                      
            while(sigue){          

                if((val[4]>2.5)&&(val[3]>3.5)&&(val[2]>umbral)){ sigue=false; }
                 
                
                if((val[0]<umbral2)&&(val[1]<umbral2)&&(val[2]<umbral2)&&(val[3]<umbral2)&&(val[4]<umbral2)){  girocompleto();  break; }      
                                       
                ledcWrite(0,0);ledcWrite(1,0);
                ledcWrite(2,0);ledcWrite(3,vel); 
                espera(1);   
                                    
            }           

              
                
        }

        ledcWrite(0,0);ledcWrite(1,vel);
        ledcWrite(2,0);ledcWrite(3,vel);
  
        deriva=0;
        
    }else {

        if(vgiro<0){  //Giro a la izquierda

           ledcWrite(0,0);ledcWrite(1,vel);
           ledcWrite(2,0);ledcWrite(3,vel2);                         
                           
        }else{   //Giro a la derecha
                                       
           ledcWrite(0,0);ledcWrite(1,vel2);
           ledcWrite(2,0);ledcWrite(3,vel);            
                
        }
  
    }

  }

  espera(50); 
  
}
