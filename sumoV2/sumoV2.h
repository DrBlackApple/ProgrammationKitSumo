//
// Created by drblackapple on 07/10/2020.
//

#ifndef SUMOV2_SUMOV2_H
#define SUMOV2_SUMOV2_H

#include <Arduino.h>

/*
 * Motors pins
 */
#define MOTOR_1_A 5
#define MOTOR_1_B 7
#define MOTOR_1_PWM 6
#define MOTOR_2_A 2
#define MOTOR_2_B 4
#define MOTOR_2_PWM 3

/*
 * Sensor pins
 */
// ultrason
#define UTS1_OUT 8
#define UTS1_READ A4
#define UTS2_OUT 12
#define UTS2_READ A5

// infrarouge
#define IR1 A6
#define IR2 A7

// ligne blanche
#define LB1 A2
#define LB2 A3

/*
 * Servo moteur
 */
#define SERVO1 9
#define SERVO2 10

/*
 * boutons
 */
#define BT1 A0
#define BT2 A1

/*
 * leds & autre
 */
#define LED1 11
#define LED_ARDUINO 13



/* ########### Functions ########### */

/*
 * A utiliser dans la fonctions setup()
 * Initialise les pins des capteurs
 */
void initSensor()
{
    pinMode(UTS1_OUT, OUTPUT);
    pinMode(UTS2_OUT, OUTPUT);
    digitalWrite(UTS2_OUT, LOW);
    digitalWrite(UTS1_OUT, LOW);
    pinMode(UTS1_READ, INPUT);
    pinMode(UTS2_READ, INPUT);
    pinMode(IR1, INPUT);
    pinMode(IR2, INPUT);
    pinMode(LB1, INPUT);
    pinMode(LB2, INPUT);
}

/*
 * Initialise le pont en H
 */
void initMotors()
{
    pinMode(MOTOR_1_A, OUTPUT);
    pinMode(MOTOR_1_B, OUTPUT);
    pinMode(MOTOR_1_PWM, OUTPUT);
    pinMode(MOTOR_2_PWM, OUTPUT);
    pinMode(MOTOR_2_A, OUTPUT);
    pinMode(MOTOR_2_B, OUTPUT);
}

/*
 * Lit le capteur ultrason demandé
 * retourne la valeur en mm
 */
unsigned long readUts(uint8_t trigg_pin, uint8_t read_pin)
{
    digitalWrite(trigg_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigg_pin, LOW);
    return pulseIn(read_pin, HIGH, 25000UL) * 5.8; // récupère la durée de l'impulsion en µs puis converti en mm (v = d/t)
}

/*
 * Lit le capteur ultrason 1
 */
unsigned long readUts1()
{
    return readUts(UTS1_OUT, UTS1_READ);
}

/*
 * Lit le capteur ultrason 2
 */
unsigned long readUts2()
{
    return readUts(UTS2_OUT, UTS2_READ);
}

#define RIGHT 0
#define LEFT 1
#define FORWARD 2
#define BACKWARD 3
/*
 * change la vitesse des moteurs 1 par 1
 */
void change_motor_speed(uint8_t speed1, uint8_t speed2)
{
    analogWrite(MOTOR_1_PWM, speed1);
    analogWrite(MOTOR_2_PWM, speed2);
}

/*
 * Fait bouger les moteurs selon un sens et une vitesse sans les fonctions digitalWrite pour être plus rapide
 */
void move_robot(uint8_t direction, uint8_t speed = 255)
{
    PORTD |= 0b10110100; // met les 4 pins de contrôles à HIGH
    switch (direction) {
        //à droite
        case RIGHT:
            /*digitalWrite(MOTOR_1_A, HIGH);
            digitalWrite(MOTOR_1_B, LOW);
            digitalWrite(MOTOR_2_A, LOW);
            digitalWrite(MOTOR_2_B, HIGH);*/
            PORTD ^= 0b10000100;
            break;

        //à gauche
        case LEFT:
            /*digitalWrite(MOTOR_1_A, LOW);
            digitalWrite(MOTOR_1_B, HIGH);
            digitalWrite(MOTOR_2_A, HIGH);
            digitalWrite(MOTOR_2_B, LOW);*/
            PORTD ^= 0b00110000;
            break;

        //tout droit
        case FORWARD:
            /*digitalWrite(MOTOR_1_A, LOW);
            digitalWrite(MOTOR_1_B, HIGH);
            digitalWrite(MOTOR_2_A, LOW);
            digitalWrite(MOTOR_2_B, HIGH);*/
            PORTD ^= 0b00010100;
            break;

        //marche arrière
        case BACKWARD:
            /*digitalWrite(MOTOR_1_A, HIGH);
            digitalWrite(MOTOR_1_B, LOW);
            digitalWrite(MOTOR_2_A, HIGH);
            digitalWrite(MOTOR_2_B, LOW);*/
            PORTD ^= 0b10100000;
            break;
    }

    change_motor_speed(speed, speed);
}

/*
 * Stoppe le robot en position bloquée
 */
void stop_block()
{
    PORTD |= 0b10110100;
    change_motor_speed(0, 0);
}

#endif //SUMOV2_SUMOV2_H
