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
#define sensor 32
<<<<<<< Updated upstream
#define btn1 25
=======
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
>>>>>>> Stashed changes

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
void funcTemp(void);

//-----------------------------------------------------------------------------
//Variabls Globales
//-----------------------------------------------------------------------------
float vaSensor = 0; // variable para el sensor
float tempC = 0;

//-----------------------------------------------------------------------------
//ISR
//-----------------------------------------------------------------------------
void IRAM_ATTR tempB1()
{
<<<<<<< Updated upstream
=======
  //lectura de sensor en loop para que continuamente este leyendo sin imprimir
  vaSensor = analogRead(sensor);           //leer el valor analógico del sensor
  tempC = vaSensor * (5.0 / 4095.0) * 100; //convertir a Centigrados
  delay(100);                              //espera

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
=======
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
>>>>>>> Stashed changes
}

//-----------------------------------------------------------------------------
//loop principal
//-----------------------------------------------------------------------------
void loop()
{
  vaSensor = analogRead(sensor);       //leer el valor analógico del sensor
  tempC = (vaSensor * 110.0) / 1024.0; //convertir a volts
  delay(100);                          //esperar 100ms
}
