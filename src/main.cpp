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
#include "esp_adc_cal.h"
//************************ Adafruit IO Config *******************************
// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "karenjoachin"
#define IO_KEY "aio_vBSs15SegBI2g6RnAUvsKvWYq6aL"

//******************************* WIFI **************************************
#define WIFI_SSID "CLARO1_DC7424"
#define WIFI_PASS "2173PCxQFM"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);

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
#define A 1  //15
#define B 15 //2
#define C 22 //16
#define D 3  //17
#define E 19
#define f 16 //3
#define G 17 //22
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
void configurarPWMServo(void);

void confignumeros(void); // para configurar numeros que van a cada display

//-----------------------------------------------------------------------------
//Variabls Globales
//-----------------------------------------------------------------------------
int cbtn1 = 0;
//sensor temp
float vaSensor = 0.0; //para leer lo que da el sensor
float tempC = 0.0;    //para temperatura final
float adcRaw = 0;
float adcFiltradoEMA = 0;
float tempReal = 0;

double alpha = 0.09;

//leds
int dutycycleLED = 100;
int dutycycleLED2 = 0;

//servo
int dutycycle = 6; // es mi angulo 0 del servo
int temperatura = 0;

//para separar la temperatura y leerlas en display
int decenas = 0;
int unidades = 0;
int decimal = 0;

//unsigned long lastimeDisplay;
unsigned long lastime;
unsigned int sampletime = 3000;

unsigned long antes;
unsigned int sampletimeLeds = 5;

unsigned long ti = 0;

//-----------------------------------------------------------------------------
//ISR
//-----------------------------------------------------------------------------
void IRAM_ATTR tempB1() //para imprimir valores solo cuando querramos
{

  if (digitalRead((btn1) == HIGH))
  {
    cbtn1++;
  }
  if (cbtn1 > 2)
  {
    cbtn1 = 0;
    Serial.print("no se envian datos ");
    Serial.print(cbtn1, 0);
  }
}

//-----------------------------------------------------------------------------
//Configuracion
//-----------------------------------------------------------------------------
// this int will hold the current count for our sketch

AdafruitIO_Feed *TempFeed = io.feed("Temp");
void setup()
{
  Serial.begin(115200);

  while (!Serial)
    ;
  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  lastime = millis();
  antes = millis();

  pinMode(btn1, INPUT_PULLUP);
  attachInterrupt(btn1, tempB1, HIGH);
  configurarPWMLED();
  configurarLED();

  configurarPWMServo();
  //configurar display y llamo de la libreria
  configurarDisplay(A, B, C, D, E, f, G, dP);

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

  if (cbtn1 == 1)
  {

    configurarLED();
    //primer display
    digitalWrite(display1, HIGH);
    digitalWrite(display2, LOW);
    digitalWrite(display3, LOW);
    desplegar7Seg(decenas);
    desplegarPunto(0);
    ti = millis();
    while (millis() < ti + 5) ;

    //segundo display
    digitalWrite(display1, LOW);
    digitalWrite(display2, HIGH);
    digitalWrite(display3, LOW);
    desplegar7Seg(unidades);
    desplegarPunto(1);
    ti = millis();
    while (millis() < ti + 5);

    //tercer display
    digitalWrite(display1, LOW);
    digitalWrite(display2, LOW);
    digitalWrite(display3, HIGH);
    desplegar7Seg(decimal);
    desplegarPunto(0);
    confignumeros();
  }
  ti = millis();
  while (millis() < ti + 5);

  if (millis() - lastime >= sampletime) //es como un delay de 3 segundos par adafuit
  {

    io.run();
    Serial.print("sending Temperatura -> ");
    Serial.println(tempReal, 1);
    Serial.print(" °C  ");
    TempFeed->save(tempReal); //guardara en la nube los datos

    Serial.print(" \n");
    Serial.print("Enviando ");
    Serial.print(cbtn1, 1);
    Serial.print(" \n");

    lastime = millis();
  }
  confignumeros();
}
else
{

  ledcWrite(PWMcanalL3, dutycycleLED2);
  ledcWrite(PWMcanalL1, dutycycleLED2);
  ledcWrite(PWMcanalL2, dutycycleLED2);
  digitalWrite(display1, LOW);
  digitalWrite(display2, LOW);
  digitalWrite(display3, LOW);
  dutycycle = 13;
  ledcWrite(PWMcanalS, dutycycle);
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
  if (tempC <= 37.0) //led verde
  {
    ledcWrite(PWMcanalL3, dutycycleLED);
    ledcWrite(PWMcanalL1, dutycycleLED2);
    ledcWrite(PWMcanalL2, dutycycleLED2);
    dutycycle = 6;
    ledcWrite(PWMcanalS, dutycycle);
  }
  else if (tempC < 37.5 && tempC > 37.0) //led amarilla
  {

    ledcWrite(PWMcanalL2, dutycycleLED);
    ledcWrite(PWMcanalL3, dutycycleLED2);
    ledcWrite(PWMcanalL1, dutycycleLED2);
    dutycycle = 13;
    ledcWrite(PWMcanalS, dutycycle);
  }
  else if (tempC >= 37.5) //led roja
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
  adcRaw = analogReadMilliVolts(sensor);
  tempC = adcRaw / 10;
  adcFiltradoEMA = (alpha * tempC) + ((1.0 - alpha) * adcFiltradoEMA);
  tempReal = adcFiltradoEMA;
  temperatura = tempReal * 10;
  decenas = temperatura / 100;
  unidades = (temperatura - (decenas * 100)) / 10;
  decimal = (temperatura - (decenas * 100) - (unidades * 10));

  ti = millis();
  while (millis() < ti + 10)
}
