//-----------------------------------------------------------------------------
//Universidad del Valle de Guatemala
//BE3015 Electronica Digital 2
//Karen Joachin
// Proyecto 1
//-----------------------------------------------------------------------------
//Librerias
//-----------------------------------------------------------------------------
#include <Arduino.h>

//-----------------------------------------------------------------------------
//Definiciond de etiquetas
//-----------------------------------------------------------------------------
//primera parte
#define sensor 32
#define btn1 25
//segunda parte
#define PWMcanalL1 1 // tenemos 16 canales del 0 a 15
#define frecPWM 50   // frecuencia en Hz para todo leds y motor.
#define resolucion 8 //8 bits se puede de 1 a 16 bits ... significa que tengo de 0 a 256 para dutty cicle
#define pinPWMR 4    //ahi va la led roja

#define PWMcanalL2 2 // tenemos 16 canales del 0 a 15
#define pinPWMA 5    //ahi va la led amarilla

#define PWMcanalL3 3 // tenemos 16 canales del 0 a 15
#define pinPWMV 18   //para led verde

//#define PWMcanalS 4    // tenemos 16 canales del 0 a 15
//#define pinPWMServo 21 //para la señal pwm al servo

//-----------------------------------------------------------------------------
//Prototipo de funciones
//-----------------------------------------------------------------------------
void IRAM_ATTR tempB1(void);
//void IRAM_ATTR ledtemp(void);

void configurarPWMLED(void);
void configurarLED(void);
//-----------------------------------------------------------------------------
//Variabls Globales
//-----------------------------------------------------------------------------
float vaSensor = 0.0; //para leer lo que da el sensor
float tempC = 0.0;    //para temperatura final

int dutycycleLED = 100;
int dutycycleLED2 = 0;

//-----------------------------------------------------------------------------
//ISR
//-----------------------------------------------------------------------------
void IRAM_ATTR tempB1() //para imprimir valores solo cuando querramos
{
  Serial.print("Temperatura = ");
  Serial.print(tempC, 1);
  Serial.print(" °C  ");
  Serial.print(" \n");
  configurarPWMLED();
}


//-----------------------------------------------------------------------------
//Configuracion
//-----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  pinMode(btn1, INPUT_PULLUP);
  attachInterrupt(btn1, tempB1, HIGH);
  //attachInterrupt(sensor, ledtemp, CHANGE);
}

//-----------------------------------------------------------------------------
//loop principal
//-----------------------------------------------------------------------------
void loop()
{
  //lectura de sensor en loop para que continuamente este leyendo sin imprimir
  //lo puse en loop porque no me sale si lo pongo en interrupcion
  vaSensor = analogRead(sensor);           //leer el valor analógico del sensor
  tempC = vaSensor * (5.0 / 4095.0) * 100; //convertir a Centigrados
  delay(1000);                             //esperar 1s

  // para leds
  configurarLED();
}




//-----------------------------------------------------------------------------
//Funcion para configurar pwm de leds
//-----------------------------------------------------------------------------
void configurarPWMLED(void)
{
  // primero se configura el modulo pwm
  ledcSetup(PWMcanalL1, frecPWM, resolucion);
  ledcSetup(PWMcanalL2, frecPWM, resolucion);
  ledcSetup(PWMcanalL3, frecPWM, resolucion);
  //Para leds. Se conecta el GPIO al canal
  ledcAttachPin(pinPWMR, PWMcanalL1);
  ledcAttachPin(pinPWMV, PWMcanalL3);
  ledcAttachPin(pinPWMA, PWMcanalL2);
}

//-----------------------------------------------------------------------------
//Funcion para configurar leds
//-----------------------------------------------------------------------------
void configurarLED(void)
{
  if (tempC <= 18.0) //led verde
  {
    ledcWrite(PWMcanalL3, dutycycleLED);
    ledcWrite(PWMcanalL1, dutycycleLED2);
    ledcWrite(PWMcanalL2, dutycycleLED2);
  }
  if (tempC < 23.5 && tempC > 18.0) //led amarilla
  {

    ledcWrite(PWMcanalL2, dutycycleLED);
    ledcWrite(PWMcanalL3, dutycycleLED2);
    ledcWrite(PWMcanalL1, dutycycleLED2);
  }
  if (tempC >= 23.5) //led roja
  {
    ledcWrite(PWMcanalL1, dutycycleLED);
    ledcWrite(PWMcanalL3, dutycycleLED2);
    ledcWrite(PWMcanalL2, dutycycleLED2);
  }
}
