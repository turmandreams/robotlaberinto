
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;



#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>

#define PIN_WS2812B 2   
#define NUM_PIXELS 1   

Adafruit_NeoPixel ws2812b(NUM_PIXELS,PIN_WS2812B,NEO_GRB + NEO_KHZ800);



TaskHandle_t Task1; 


float sinled[5];
float conled[5];
float val[5];


float filtrojames1=0.85;
float filtrojames2=1.0-filtrojames1;



////PID////

float pid;
float p=0.0;
float ii=0.0;
float d=0.0;

float  error, error_anterior;


float Kp=8.0;     //0-25.5
float Ki=0.003;   //0-25.5
float Kd=0.04;     //0-25.5

///////////


int velrecta=250;           /// 0-255   //Cuando va recto va a tope.
int velrectagiro=250;       /// 0-255
int velgiro=250;            /// 0-255
int velgirocompleto=250;    /// 0-255


float umbralpared=12.0;
float umbralparedlateral=8.0;
float umbralgirocompleto=8.0;

   
float vgiro;
int deriva=0;
boolean sigue=true;

boolean esperasensores=false;

boolean muevemotores=true;

int estado=-1;
int estadoanterior=-1;

int calibracion[5][20]={ 
                   {4094,4094,3300,2550,2000,1500,1220,980,840,730,640,575,510,450,415,375,350,320,300,275},
                   {4094,4094,4040,3130,2500,1930,1550,1270,1100,950,825,725,650,575,530,480,440,415,380,365},
                   {4094,4094,3650,2900,2350,1900,1500,1250,1060,910,800,700,625,570,510,460,425,390,360,340},
                   {4094,4000,3240,2400,1850,1470,1150,950,820,690,600,530,470,420,380,350,320,300,275,260},
                   {4094,4000,3200,2470,1880,1550,1250,1050,870,760,670,600,530,490,450,410,385,350,335,310}};


void pwm(int canal,int duty){

    int dutyc=0;
    
    do{ 
        ledcWrite(canal,duty);        
        dutyc=ledcRead(canal);
    }while(duty!=dutyc);        
       
}

void inicializapid(){
    p=0.0;
    ii=0.0;
    d=0.0;

    //pwm(0,0);pwm(1,0);
    //pwm(2,0);pwm(3,0); 

    //espera(5000);
}


void calculapid(){
    
    
    vgiro=val[0]+val[1]-val[3]-val[4];  //Los valores de vgiro varian entre 20 y -20 

    if(vgiro>20.0){ vgiro=20.0; }            ///Limitamos los valores para que no haya sorpresas.
    else if(vgiro<-20.0){ vgiro=-20.0; }  
    
    error = vgiro;
  
    p=error;
    ii=ii+error;
    if(ii>255.0){ ii=512.0;}
    else if(ii<-512.0){ ii=-512.0;}
    
    d = error-error_anterior;        // error differential 
    error_anterior = error;          // Save last error for next loop
  
    pid = (Kp*p)+(Ki*ii)+(Kd*d);  // Do PID

    pid = constrain(pid,-512,512);   

    //Serial.print(vgiro);
    //Serial.print(",");
    //Serial.println(pid);
    
}

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


void espera(int milisegundos){  
  int tiempo=millis();  
  while((millis()-tiempo)<milisegundos){                 
      delay(1);      
      wdt();        
  }
  
}



void setup(){

  Serial.begin(115200);

  SerialBT.begin("BOTMAZE"); //Bluetooth device name
  
  ws2812b.begin(); 
  
  analogReadResolution(9);

  ledcAttachPin(12,0);ledcSetup(0,256,8);
  ledcAttachPin(13,1);ledcSetup(1,256,8);
  ledcAttachPin(27,2);ledcSetup(2,256,8);
  ledcAttachPin(14,3);ledcSetup(3,256,8);
  
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

//  pinMode(12,OUTPUT);pinMode(13,OUTPUT);
//  pinMode(14,OUTPUT);pinMode(27,OUTPUT);

  pwm(0,0);pwm(1,0);
  pwm(2,0);pwm(3,0);
  
   
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

     
    
   espera(60000);

  
   
}



void Task1code( void * pvParameters ){    // Lectura sensore IR

  int tiempo=millis();
  
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


     esperasensores=true;     
     for(int i=0;i<5;i++){      
          int vall=((int)(conled[i]-sinled[i]))*8;             
          val[i]=normaliza(vall,i);                 
     }

     String dato="0,";
     for(int i=0;i<5;i++){          
              dato+=val[i];
              dato+=","; 
     }

     dato+="22"; 
     SerialBT.println(dato);
      
     esperasensores=false;
      
     wdt();    

     if((millis()-tiempo)>100){

        if(estado!=estadoanterior){

            if(estado==0){  ws2812b.setPixelColor(0,ws2812b.Color(0,0,50)); }
            else if(estado==1){  ws2812b.setPixelColor(0,ws2812b.Color(50,0,0)); }
            else if(estado==2){  ws2812b.setPixelColor(0,ws2812b.Color(0,50,0)); }
            else if(estado==3){  ws2812b.setPixelColor(0,ws2812b.Color(50,50,50)); }

            ws2812b.show();
            estadoanterior=estado;
        }

        tiempo=millis();
     }
    
  } 
  
}

void loop(){

    
  if(muevemotores){
  
    //int val=(int)(conled[2]-sinled[2]); 

    while(esperasensores){ espera(1); }    
    vgiro=val[0]+val[1]-val[3]-val[4];

    boolean recto=true;

    if(val[2]>=umbralpared){ recto=true; }
    else{ recto=false; }

    if((!recto)&&((val[0]>=umbralparedlateral)||(val[0]>=umbralparedlateral))){ recto=false; }
    else { recto=true; }

    if((val[0]<umbralgirocompleto)&&(val[1]<umbralgirocompleto)&&(val[2]<umbralgirocompleto)&&(val[3]<umbralgirocompleto)&&(val[4]<umbralgirocompleto)){   ///Giro Completo
        
        while(esperasensores){ espera(1); }    
        vgiro=val[0]+val[1]-val[3]-val[4];

        estado=3;
        
        if(vgiro<0){  //Giro a la izquierda
      
            pwm(0,0);pwm(1,velgirocompleto);
            pwm(3,0);pwm(2,velgirocompleto);
      
            delay(380);
      
            pwm(0,0);pwm(1,velrectagiro);
            pwm(2,0);pwm(3,velrectagiro);
      
            delay(100);
                             
        }else{   //Giro a la derecha
      
            pwm(1,0);pwm(0,velgirocompleto);
            pwm(2,0);pwm(3,velgirocompleto);             
                 
            delay(380);
      
      
            pwm(0,0);pwm(1,velrectagiro);
            pwm(2,0);pwm(3,velrectagiro);
      
            delay(200);
                            
        }
                     
        deriva=0;
        inicializapid();
 
    
    }else if(recto){    ///No hay pared cerca y va "recto"

        estado=0;
        
        calculapid();
        
        if(pid<0){  //Giro a la izquierda

            if(pid>-256){
                int v=map(pid,-256,0,0,velrecta);
                pwm(0,0);pwm(1,velrecta);
                pwm(2,0);pwm(3,v);                                                                
            }else{
                int v=map(pid,-256,-512,0,velrecta);
                pwm(0,0);pwm(1,velrecta);
                pwm(3,0);pwm(2,v);      
            }
                           
        }else{   //Giro a la derecha

            if(pid<256){
                int v=map(pid,0,256,velrecta,0);
                pwm(0,0);pwm(1,v);
                pwm(2,0);pwm(3,velrecta);                                                                
            }else{
                int v=map(pid,256,512,0,velrecta);
                pwm(1,0);pwm(0,v);
                pwm(2,0);pwm(3,velrecta);      
            }                                       
                      
        }

        
    }else{    ///Giro de esquina

        sigue=true;  

        while(esperasensores){ espera(1); }    
        vgiro=val[0]+val[1]-val[3]-val[4];

        if(vgiro<0){  //Giro a la izquierda

            estado=1;
        
            pwm(0,0);pwm(1,velrectagiro);
            pwm(3,0);pwm(2,velgiro);

            delay(150);

            pwm(0,0);pwm(1,velrectagiro);
            pwm(2,0);pwm(3,velrectagiro);

            delay(100);            
             
        }else{   //Giro a la derecha

            estado=2;
            
            pwm(1,0);pwm(0,velgiro);
            pwm(2,0);pwm(3,velrectagiro);             
           
            delay(150);

            pwm(0,0);pwm(1,velrectagiro);
            pwm(2,0);pwm(3,velrectagiro);

            delay(200);
            
        }
               
        deriva=0;
        inicializapid();

       
        
    }

  }

  //espera(10); 
  
}
