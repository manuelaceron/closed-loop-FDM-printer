//DESARROLLO DE UN SISTEMA DE CONTROL EN LAZO CERRADO PARA LOS MOVIMIENTOS DEL EXTRUSOR Y LA PLATAFORMA DEL SISTEMA DE IMPRESIÓN UAO 3DP
//Manuela Cerón Viveros


//////////////////////////SECCIÓN 1: Definición de librerías y variables//////////////////////////
#include <digitalWriteFast.h>

//Entrada señal desde encoder eje X, canal A y B
#define encoderXCA  2 //Interrupción 0
#define encoderXCB  8  

// Entrada señal desde encoder eje Y, canal A y B
#define encoderYCA  19 //Interrupción 3
#define encoderYCB  11

// Entrada señal desde encoder eje Z, canal A y B
#define encoderZCA  20 //Interrupción 4
#define encoderZCB  12  

//Entrada señal Step y Dir desde RAMPS para driver 1
#define stepD1Ramps 21 //Interrupción 5
#define dirD1Ramps 9  

// Entrada señal Step y Dir desde RAMPS para driver 2
#define stepD2Ramps 18 //Interrupción 2
#define dirD2Ramps 13

// Entrada señal Step y Dir desde RAMPS para driver 3
#define stepD3Ramps 3 //Interrupción 1
#define dirD3Ramps 10 

//Salida señal Step y Dir hacia motor 1
#define stepM1 22 
#define dirM1 23 

//Salida señal Step y Dir hacia motor 2
#define stepM2 28
#define dirM2 29

//Salida señal Step y Dir hacia motor 3
#define stepM3 24
#define dirM3 25

//Variables para almacenar la lectura de los encoder
volatile long encoder0PosX = 0;
volatile long encoder0PosY = 0;
volatile long encoder0PosZ = 0;

volatile float PosEncoderx =0;
volatile float PosEncodery =0;
volatile float PosEncoderz =0;

//Variables para almacenar la posición de los encoder convertida a pulsos
volatile int PosEnPulD1 = 0;
volatile int PosEnPulD2 =0;
volatile int PosEnPulD3 =0;

 //Variables para almacenar la posición de los encoders en mm
double x;
double y;
double z;

// Variables para almacenar los pulsos enviados desde RAMPS a cada uno de los drivers
volatile float pasosD1=0; 
volatile float pasosD2=0;
volatile float pasosD3=0; 

//Variables para almacenar la posición deseada
volatile float Posx=0;
volatile float Posy=0;
volatile float Posz=0;

//Variables para almacenar el error calculado en cada driver
int errorD1=0;
int errorD2=0;
int errorD3=0;

// Error: Cantidad de pulsos permitidos 
int ep= 10; 

//Factor resolución ejes x y y
double fr= 80.5;

//factor resolución eje z
double frz= 252.0;

//Delay en micro segundos para enviar pulsos a los motores
int dl=5;
int d= 5; 

//Variables para definir si se trabaja en lazo abierto o cerrado
int input;
bool lazoC = false;


 

void setup() { 
 Serial.begin (250000);
 Serial.println("Lazco Abierto [1] - Lazo Cerrado [2] ");

//////////////////////SECCIÓN 2: Definición de modo E/S de las señales //////////////////////

  //Se define el modo salida de las señales que se envían a los motores
  pinModeFast(stepM1, OUTPUT); 
  pinModeFast(dirM1, OUTPUT);  
  pinModeFast(stepM2, OUTPUT); 
  pinModeFast(dirM2, OUTPUT);  
  pinModeFast(stepM3, OUTPUT); 
  pinModeFast(dirM3, OUTPUT);
  
  //Se define el modo de entrada de las señales recibidas desde la RAMPS
  pinModeFast(dirD1Ramps, INPUT); 
  pinModeFast(dirD2Ramps, INPUT);  
  pinModeFast(dirD3Ramps, INPUT);
  
//////////////////////////SECCIÓN 3: Definición de interrupciones//////////////////////////

// Interrupción 0: Recibe los pulsos del Encoder X por el Canal A (Pin 2) 
attachInterrupt(0, lecturaEncoderX, CHANGE);  

// Interrupción 1: Recibe los pulsos del STEMP M3 desde firmware (Pin 3)
attachInterrupt(1, lecturaSTEPM3, FALLING); 

// Interrupción 2: Recibe los pulsos del STEMP M1 desde firmware (Pin 18)
attachInterrupt(2, lecturaSTEPM1, FALLING); 

// Interrupción 3: Recibe los pulsos del Encoder Y por el Canal A (Pin 19) 
attachInterrupt(3, lecturaEncoderY, CHANGE);  

// Interrupción 4: Recibe los pulsos del Encoder Z por el Canal A (Pin 20)
attachInterrupt(4, lecturaEncoderZ, CHANGE); 

// Interrupción 5: Recibe los pulsos del STEMP M2 desde firmware (Pin 21) 
attachInterrupt(5, lecturaSTEPM2, FALLING);  
} 

void loop(){

//////////////////////////SECCIÓN 4: Definición de interrupciones//////////////////////////
 
  input=Serial.read();
  input=input-48;
  if(input==1){
    lazoC= false;
    Serial.println("Lazo Abierto");
  }

// Lazo abierto: Activar las tres palancas del sistema de control en el lado de lazo abierto
  if(input==2){
    lazoC= true;
    Serial.println("Lazo Cerrado");
// Lazo cerrado: Activar las tres palancas del sistema de control en el lado de lazo cerrado
  }

////////////////SECCIÓN 5: Posición actual (Lectura encoders->Posición (mm))//////////////////

x= (PosEncoderx)/200;
y= (PosEncodery)/200;
z= (350.0)-(PosEncoderz/200);

////////////////SECCIÓN 6: Posición actual (Posición (mm)->Pulsos por driver)//////////////////

// Utilizando las ecuaciones que describen el movimiento de la impresora, y a partir de la posición actual de cada driver, se puede calcular la cantidad de pulsos actuales enviados desde la RAMPS a cada driver. De esta manera se puede comparar en términos de pulsos la posición deseada y actual

PosEnPulD1=fr*(x-y);
PosEnPulD2=fr*(-x-y);
PosEnPulD3= 22664-(z*frz);

/////////////SECCIÓN 7: Posición deseada (Pulsos por driver->Posición deseada)////////////////

//Los pulsos que se envían desde la RAMPS para cada driver, se transforman en la posición deseada (mm) que se establece desde el control manual del firmware o desde el código g. Para transformar pulsos en posición se utilizan las ecuaciones que describen el movimiento de la impresora.
Posx= (pasosD1-pasosD2)/(fr*2);
Posy= (-pasosD1-pasosD2)/(fr*2);
Posz= 350.0-(pasosD3/frz);
  
/////////////////////SECCIÓN 8: Cálculo del error por driver (en pulsos)/////////////////////////
errorD1= (pasosD1) - (PosEnPulD1);
errorD2= (pasosD2) - (PosEnPulD2);
errorD3= (pasosD3) - (PosEnPulD3);

//////////////////////////////SECCIÓN 9: Control de posición//////////////////////////////////

//Si el error de cada driver se encuentra fuera del rango de error permitido (–ep y ep), se envía una secuencia para contrarrestar el error

  if(errorD1 > ep || errorD1 <-ep )
  {
    if(lazoC==true){
      moverD1();
      }
  }

  if(errorD2 > ep || errorD2 <-ep )
  {
   if(lazoC==true){
    moverD2();
    }
  }

  if(errorD3 > ep || errorD3 <-ep )
  {
    if(lazoC==true){
      moverD3();
      }
  }

/////////////////////////////SECCIÓN 10: Impresión de variables/////////////////////////////////

 if(lazoC==true){
  //Posición actual (mm) – Posición deseada (mm) – Error por driver
    Serial.print(" Encx: ");
    Serial.print(x);
    Serial.print(" Ency: ");
    Serial.print(y);
    Serial.print(" Encz: ");
    Serial.print(z);
    Serial.print(" X: ");
    Serial.print(Posx);
    Serial.print(" Y: ");
    Serial.print(Posy);
    Serial.print(" Z: ");
    Serial.print(Posz);
    Serial.print(" errorD1: ");
    Serial.print(errorD1);
    Serial.print(" ErrorD2: ");
    Serial.print(errorD2);
    Serial.print(" ErrorD3: ");
    Serial.println(errorD3);
  }  
}

/////////////////////////////SECCIÓN 11: Posición actual /////////////////////////////////

static unsigned char New, Old, NewY, OldY, NewZ, OldZ;

// Matriz de cuadratura
//const int QEM [16] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};
//const int QEM [16] = {0,0,0,-1,0,0,-1,0,0,-1,0,0,-1,0,0,0}; 
const int QEM [16] = {0,-1,1,-2,1,0,2,-1,-1,2,0,1,-2,1,-1,0}; 

//Lectura de cada encoder

void lecturaEncoderX(){
  Old = New;
  New = digitalReadFast(encoderXCA) + 2*digitalReadFast(encoderXCB); 
  encoder0PosX-= QEM [Old*4 + New];
  PosEncoderx=encoder0PosX; 
}

void lecturaEncoderY(){
  OldY = NewY;
  NewY = digitalReadFast(encoderYCA)*2 + digitalReadFast(encoderYCB); 
  encoder0PosY-= QEM [OldY * 4 + NewY];
  PosEncodery=encoder0PosY; 
}

void lecturaEncoderZ(){
  OldZ = NewZ;
  NewZ = digitalReadFast(encoderZCA) + 2*digitalReadFast(encoderZCB); 
  encoder0PosZ-= QEM [OldZ * 4 + NewZ];
  PosEncoderz=encoder0PosZ; 
}

/////////////////////////////SECCIÓN 12: Posición deseada /////////////////////////////////

// El sistema actúa como un puente, recibe los pulsos de la RAMPS para cada driver, los lee (insumo para obtener posición deseada) e inmediatamente los envía a los drivers para que se pueda generar el movimiento de cada motor

//Los pulsos que la RAMPS envía al driver 1, se envían a motor 2
void lecturaSTEPM1(){

//Lazo cerrado: Acumulo pulso y lo envío al motor 2
  if (lazoC==true){
    if (digitalReadFast(dirD1Ramps)){ // x+
      pasosD1++;
      digitalWriteFast(dirM2, HIGH); //M2 CCW
      digitalWriteFast(stepM2, HIGH);
      delayMicroseconds(dl);
      digitalWriteFast(stepM2, LOW);
      }               
      else {
        pasosD1--;
        digitalWriteFast(dirM2, LOW); //M2 CW
        digitalWriteFast(stepM2, HIGH);
        delayMicroseconds(dl);
        digitalWriteFast(stepM2, LOW);
        }
        }
                    
}
   
//Los pulsos que la RAMPS envía al driver 2, se envían a motor 1

void lecturaSTEPM2(){ 
  //Lazo cerrado: Acumulo pulso y lo envío al motor 1
  if (lazoC==true){
    if (digitalReadFast(dirD2Ramps)){
      pasosD2--;
      digitalWriteFast(dirM1, HIGH); //M1 CW
      digitalWriteFast(stepM1, HIGH);
      delayMicroseconds(dl);
      digitalWriteFast(stepM1, LOW);
      }            
      else {
        pasosD2++;
        digitalWriteFast(dirM1, LOW); //M1 CCW
        digitalWriteFast(stepM1, LOW);
        delayMicroseconds(dl);
        digitalWriteFast(stepM1, HIGH);
        }
        }
}

//Los pulsos que la RAMPS envía al driver 3, se envían a motor 3 y 4

void lecturaSTEPM3(){ 

//Lazo cerrado: Acumulo pulso y lo envío al motor 3 y 4
  if (lazoC==true){
    if (digitalReadFast(dirD3Ramps)){
      pasosD3--;
      digitalWriteFast(dirM3, HIGH); //M3 CW
      digitalWriteFast(stepM3, HIGH);
      delayMicroseconds(dl);
      digitalWriteFast(stepM3, LOW);
      }
      else {
        pasosD3++;
        digitalWriteFast(dirM3, LOW); //M3 CCW
        digitalWriteFast(stepM3, LOW);
        delayMicroseconds(dl);
        digitalWriteFast(stepM3, HIGH);
        }
        }           
}

//////////////////////SECCIÓN 13: Métodos para contrarrestar el error//////////////////////////

void moverD1(){
//Si el error del driver 1 es positivo, se envía un pulso al motor 2 en sentido contrario de las manecillas del reloj (CCW)
  if (errorD1>0){
    digitalWriteFast(dirM2, HIGH); //M2 CCW
    digitalWriteFast(stepM2, HIGH);
    delayMicroseconds(d);
    digitalWriteFast(stepM2, LOW);              
    }
    
//Si el error del driver 1 es negativo, se envía un pulso al motor 2 en sentido de las manecillas del reloj (CW)
    else if(errorD1<0){
      digitalWriteFast(dirM2, LOW); //M2 CW
      digitalWriteFast(stepM2, HIGH);
      delayMicroseconds(d);
      digitalWriteFast(stepM2, LOW); 
      }
}

void moverD2(){
//Si el error del driver 2 es positivo, se envía un pulso al motor 1 en sentido contrario de las manecillas del reloj (CCW)
  if (errorD2>0){
    digitalWriteFast(dirM1, LOW); //M1 CCW
    digitalWriteFast(stepM1, LOW);
    delayMicroseconds(d);
    digitalWriteFast(stepM1, HIGH);              
    }
    
//Si el error del driver 2 es negativo, se envía un pulso al motor 1 en sentido de las manecillas del reloj (CW)
    else if(errorD2<0){
      digitalWriteFast(dirM1, HIGH); //M1 CW
      digitalWriteFast(stepM1, HIGH);
      delayMicroseconds(d);
      digitalWriteFast(stepM1, LOW); 
      }
}

void moverD3(){
//Si el error del driver 3 es positivo, se envía un pulso al motor 3 y 4 en sentido contrario de las manecillas del reloj (CCW)
  if (errorD3>0){
    digitalWriteFast(dirM3, LOW); //M3 CCW
    digitalWriteFast(stepM3, LOW);
    delayMicroseconds(d);
    digitalWriteFast(stepM3, HIGH);              
    }
    
//Si el error del driver 3 es negativo, se envía un pulso al motor 3 y 4 en sentido de las manecillas del reloj (CW)
    else if(errorD3<0){
      digitalWriteFast(dirM3, HIGH); //M3 CW
      digitalWriteFast(stepM3, HIGH);
      delayMicroseconds(d);
      digitalWriteFast(stepM3, LOW); 
      }
}



