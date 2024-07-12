
float sinled[5];
float conled[5];

int tiempo=0;



void setup(){

  
  Serial.begin(115200);

  analogReadResolution(9);

  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(14,OUTPUT);
  pinMode(27,OUTPUT);

  

  digitalWrite(12,LOW);digitalWrite(13,LOW);  
  digitalWrite(14,LOW);digitalWrite(27,LOW);
  
 
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

  tiempo=millis();

  
  
}


void loop(){
  
  digitalWrite(15,LOW);delay(1);
  
  int valor=analogRead(26);
  //sinled[0]=(sinled[0]*0.99)+(((float)valor)*0.01);
  sinled[0]=(float)valor;
  
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
  //conled[0]=(conled[0]*0.99)+(((float)valor)*0.01);
  conled[0]=(float)valor;
  
  valor=analogRead(25);
  conled[1]=(conled[1]*0.99)+(((float)valor)*0.01);

  valor=analogRead(33);
  conled[2]=(conled[2]*0.99)+(((float)valor)*0.01);

  valor=analogRead(32);
  conled[3]=(conled[3]*0.99)+(((float)valor)*0.01);

  valor=analogRead(35);
  conled[4]=(conled[4]*0.99)+(((float)valor)*0.01);

 
  if((millis()-tiempo)>100){

      String dato="0,";
    
      for(int i=0;i<5;i++){
          int val=(int)(conled[i]-sinled[i]); 
          dato+=val;
          dato+=",";     
      }
      
      dato+="2200";     
      
      //Serial.println(dato);
    
      int val=(int)(conled[0]-sinled[0]);
      
      Serial.println(val);
      
      tiempo=millis();
  }
  
}
