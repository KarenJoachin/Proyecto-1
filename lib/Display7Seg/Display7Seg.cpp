#include "Display7Seg.h"

uint8_t pinA, pinB, pinC, pinD, pinE, pinf, pinG, pindP;

//Funcion para configurar display de 7 segmentos
void configurarDisplay(uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint8_t E, uint8_t f, uint8_t G, uint8_t dP)
{
    pinA = A;
    pinB = B;
    pinC = C;
    pinD = D;
    pinE = E;
    pinf = f;
    pinG = G;
    pindP = dP;

    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(pinC, OUTPUT);
    pinMode(pinD, OUTPUT);
    pinMode(pinE, OUTPUT);
    pinMode(pinf, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pindP, OUTPUT);

    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinf, LOW);
    digitalWrite(pinG, LOW);
    digitalWrite(pindP, LOW);
}

//Funcion pasa desplegar digito de 7 segemntos
void desplegar7Seg(uint8_t digito)
{
    switch (digito)
    {
    case 0:
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, HIGH);
        digitalWrite(pinE, HIGH);
        digitalWrite(pinf, HIGH);
        digitalWrite(pinG, LOW);

        break;

    case 1:
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, LOW);
        digitalWrite(pinE, LOW);
        digitalWrite(pinf, LOW);
        digitalWrite(pinG, LOW);

        break;

    case 2:
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
        digitalWrite(pinC, LOW);
        digitalWrite(pinD, HIGH);
        digitalWrite(pinE, HIGH);
        digitalWrite(pinf, LOW);
        digitalWrite(pinG, HIGH);

        break;

    case 3:
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, HIGH);
        digitalWrite(pinE, LOW);
        digitalWrite(pinf, LOW);
        digitalWrite(pinG, HIGH);

        break;

    case 4:
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, LOW);
        digitalWrite(pinE, LOW);
        digitalWrite(pinf, HIGH);
        digitalWrite(pinG, HIGH);

        break;

    case 5:
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, HIGH);
        digitalWrite(pinE, LOW);
        digitalWrite(pinf, HIGH);
        digitalWrite(pinG, HIGH);

        break;

    case 6:
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, HIGH);
        digitalWrite(pinE, HIGH);
        digitalWrite(pinf, HIGH);
        digitalWrite(pinG, HIGH);

        break;

    case 7:
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, LOW);
        digitalWrite(pinE, LOW);
        digitalWrite(pinf, LOW);
        digitalWrite(pinG, LOW);

        break;

    case 8:
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, HIGH);
        digitalWrite(pinE, HIGH);
        digitalWrite(pinf, HIGH);
        digitalWrite(pinG, HIGH);

        break;
    case 9:
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, HIGH);
        digitalWrite(pinC, HIGH);
        digitalWrite(pinD, HIGH);
        digitalWrite(pinE, LOW);
        digitalWrite(pinf, HIGH);
        digitalWrite(pinG, HIGH);

        break;

    default:
        break;
    }
}

//Funcion para desplegar el punto
void desplegarPunto(boolean punto)
{
    if (punto == 1)
    {
        digitalWrite(pindP, HIGH);
    }
    else
    {
        digitalWrite(pindP, LOW);
    }
}