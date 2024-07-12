
float sinled[5];
float conled[5];

int tiempo=0;

///Inicializamos la calibracion con los valores obtenidos

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

void setup(){

  
  Serial.begin(115200);

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

 
  if((millis()-tiempo)>200){

      String dato="0,";
    
      for(int i=0;i<5;i++){
          int val=(int)(conled[i]-sinled[i]); 
          dato+=val;
          dato+=",";     
      }
      
      dato+="2200";     
      
      //Serial.println(dato);
    
      int val=(int)(conled[0]-sinled[0]);
      
      float normalizado=normaliza(val,2);
    
      Serial.print(val);
      Serial.print(",");
      
      Serial.println(normalizado);

      tiempo=millis();
  }
  
}
