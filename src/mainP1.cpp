//-----------------------------------------------------------------------------
//Universidad del Valle de Guatemala
//BE3015 Electronica Digital 2
//Karen Joachin
// Proyecto 1
//-----------------------------------------------------------------------------
//Librerias
//-----------------------------------------------------------------------------
#include <Arduino.h>
#include "Display7Seg.h"
//-----------------------------------------------------------------------------
//Definiciond de etiquetas
//-----------------------------------------------------------------------------
//primera parte
#define sensor 32
#define btn1 12
//segunda parte
#define PWMcanalL1 1 // tenemos 16 canales del 0 a 15
#define frecPWM 50   // frecuencia en Hz para todo leds y motor.
#define resolucion 8 //8 bits se puede de 1 a 16 bits ... significa que tengo de 0 a 256 para dutty cicle
#define pinPWMR 4    //ahi va la led roja

#define PWMcanalL2 2 // tenemos 16 canales del 0 a 15
#define pinPWMA 5    //ahi va la led amarilla

#define PWMcanalL3 3 // tenemos 16 canales del 0 a 15
#define pinPWMV 18   //para led verde

#define PWMcanalS 4    // tenemos 16 canales del 0 a 15
#define pinPWMServo 21 //para la señal pwm al servo

//display7seg
#define A 15
#define B 2
#define C 16
#define D 17
#define E 19
#define F 3
#define G 22
#define dP 23
//defino los 3 displays
#define display1 26
#define display2 27
#define display3 14
//-----------------------------------------------------------------------------
//Prototipo de funciones
//-----------------------------------------------------------------------------
void IRAM_ATTR tempB1(void);
//void IRAM_ATTR ledtemp(void);

void configurarPWMLED(void);
void configurarLED(void);

void confignumeros(void);// para configurar numeros que van a cada display

//-----------------------------------------------------------------------------
//Variabls Globales
//-----------------------------------------------------------------------------
//sensor temp
float vaSensor = 0.0; //para leer lo que da el sensor
float tempC = 0.0;    //para temperatura final
//leds
int dutycycleLED = 100;
int dutycycleLED2 = 0;

//servo
int dutycycle = 6; // es mi angulo 0 del servo
int temperatura = 0;
int decenas = 0;
int unidades = 0;
int decimal = 0;

//-----------------------------------------------------------------------------
//ISR
//-----------------------------------------------------------------------------
void IRAM_ATTR tempB1() //para imprimir valores solo cuando querramos
{
  //lectura de sensor en loop para que continuamente este leyendo sin imprimir
  vaSensor = analogRead(sensor);           //leer el valor analógico del sensor
  tempC = vaSensor * (5.0 / 4095.0) * 100; //convertir a Centigrados
  delay(100);                              //espera

  Serial.print("Temperatura = ");
  Serial.print(tempC, 1);
  Serial.print(" °C  ");
  Serial.print(" \n");
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
  configurarPWMLED();
  configurarLED();
  configurarPWMServo();
  //configurar display y llamo de la libreria
  configurarDisplay(A, B, C, D, E, F, G, dP);

  pinMode(display1, OUTPUT);
  pinMode(display2, OUTPUT);
  pinMode(display3, OUTPUT);

  digitalWrite(display1, LOW);
  digitalWrite(display2, LOW);
  digitalWrite(display3, LOW);
}

//-----------------------------------------------------------------------------
//loop principal
//-----------------------------------------------------------------------------
void loop()
{
  // para leds
  configurarLED();
  confignumeros();

//primer display
  digitalWrite(display1,HIGH);
  digitalWrite(display2,LOW);
  digitalWrite(display3,LOW);
  desplegar7Seg(decenas);
  desplegarPunto(0);
  delay(20);

//segundo display
  digitalWrite(display1,LOW);
  digitalWrite(display2,HIGH);
  digitalWrite(display3,LOW);
  desplegar7Seg(unidades);
  desplegarPunto(1);
  delay(20);

//tercer display
  digitalWrite(display1,LOW);
  digitalWrite(display2,LOW);
  digitalWrite(display3,HIGH);
  desplegar7Seg(decimal);
  desplegarPunto(0);
  delay(20);
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
//Funcion para configurar leds y motor
//-----------------------------------------------------------------------------
void configurarLED(void)
{
  if (tempC <= 19.5) //led verde
  {
    ledcWrite(PWMcanalL3, dutycycleLED);
    ledcWrite(PWMcanalL1, dutycycleLED2);
    ledcWrite(PWMcanalL2, dutycycleLED2);
    dutycycle = 6;
    ledcWrite(PWMcanalS, dutycycle);
  }
  if (tempC < 22.5 && tempC > 19.5) //led amarilla
  {

    ledcWrite(PWMcanalL2, dutycycleLED);
    ledcWrite(PWMcanalL3, dutycycleLED2);
    ledcWrite(PWMcanalL1, dutycycleLED2);
    dutycycle = 13;
    ledcWrite(PWMcanalS, dutycycle);
  }
  if (tempC >= 22.5) //led roja
  {
    ledcWrite(PWMcanalL1, dutycycleLED);
    ledcWrite(PWMcanalL3, dutycycleLED2);
    ledcWrite(PWMcanalL2, dutycycleLED2);
    dutycycle = 28;
    ledcWrite(PWMcanalS, dutycycle);
  }
}

//-----------------------------------------------------------------------------
//Funcion para configurar pwm del servo
//-----------------------------------------------------------------------------
void configurarPWMServo(void)
{ // esto lo puede haber puesto en el setup pero lo prefiero mas sordenado
  //Paso 1 configurar el modulo PWM
  ledcSetup(PWMcanalS, frecPWM, resolucion); // esta es el del servo faltan las de las leds
  //paso 2 seleccionar en que GPIO tendremos nuestras se;al de PWM
  ledcAttachPin(pinPWMServo, PWMcanalS); //para cambir ciclo de trabajo del servo faltan las de las leds
}

//-----------------------------------------------------------------------------
//Funcion para configurar lectura de sensor en deceneas, unidades, decimal
//-----------------------------------------------------------------------------
void confignumeros(void)
{
  temperatura = tempC * 10; 
  decenas = temperatura / 100; 
  unidades = (temperatura - (decenas * 100))/10;
  decimal = (temperatura-(unidades*10));
}
