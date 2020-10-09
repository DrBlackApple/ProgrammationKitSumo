#include <Arduino.h>
#include "sumoV2.h"

//initialisations
void setup() {
    Serial.begin(9600);

    initMotors();
    initSensor();
}

//boucle principale
void loop() {
    

}
