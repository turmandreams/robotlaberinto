#include <WifiEspNowBroadcast.h>
#include <WiFi.h>


TaskHandle_t Task1;  // declaramos para poder ejecutar en otro core

int tiempo=0;

int direccion=0;

void processRx(const uint8_t mac[WIFIESPNOW_ALEN], const uint8_t* buf, size_t count, void* arg){

   
    tiempo=millis();
	   
    direccion=0;
      
    if(buf[0]>190){
	      direccion=1;    
    }else if(buf[0]<60){
        direccion=2;
    }

    if((direccion==1)||(direccion==2)){
	
    	if(buf[3]<60) { direccion=3; }
            
    	else if(buf[3]>190) { direccion=4;}
    
            
    }    

    return;
}




void setup(){

  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);
  pinMode(27,OUTPUT);
  
   WiFi.persistent(false);
  bool ok = WifiEspNowBroadcast.begin("MANDO",3);
  if (!ok) {
     // Serial.println("WifiEspNowBroadcast.begin() failed");
      ESP.restart();
  }
  
  WifiEspNowBroadcast.onReceive(processRx, nullptr);

   xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

  delay(200);

}

void Task1code( void * pvParameters ){   
  
  for(;;){         

    //Serial.println("Core 2");
    WifiEspNowBroadcast.loop();       

    if((millis()-tiempo)>500) {direccion=0;tiempo=millis(); }
    yield();      
    delay(20);
        
  } 
}

void loop(){

  digitalWrite(12,LOW);digitalWrite(13,LOW);  
  digitalWrite(14,LOW);digitalWrite(27,LOW);
  
  if(direccion==1){ //
	  digitalWrite(13,HIGH);  
	  digitalWrite(27,HIGH);
  }else if(direccion==2){ //
	  digitalWrite(12,HIGH);  
	  digitalWrite(14,HIGH);
  }else if(direccion==3){ //
	  digitalWrite(13,HIGH);  
	  digitalWrite(14,HIGH);
  }else if(direccion==4){ //
	  digitalWrite(12,HIGH);  
	  digitalWrite(27,HIGH);
  }
  
  
  delay(100);
  
}
