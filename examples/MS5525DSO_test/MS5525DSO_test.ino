#include "MS5525DSO.h"
#include <Wire.h>

MS5525DSO sensor_pres(pp005GS); // Se usa el Wire por omisión

void setup() {
  // La siguiente demora es sólo para comodidad de desarrollo para enchufar el USB
  // y verlo en gtkterm. No es en lo absoluto necesaria como algoritmo requerido.
  delay(3000);
  Serial.begin(115200);

  Wire.begin(); // Se usa el SDA y SCL por omisión de la plataforma (21 y 22 para NodeMCU-32S)

  if (!sensor_pres.begin()) {
    Serial.println("FATAL: no se pudo inicializar sensor de presión!");
    while (true) delay(100);
  }

  Serial.println("INFO: sensor inicializado, volcando coeficientes...");
  sensor_pres.dumpCoefficients(Serial);
}

void loop() {
  double pressure, temperature;
  
  delay(200);

  if (!sensor_pres.readPressureAndTemperature(&pressure, &temperature)) {
    Serial.println("ERR: no se pudo leer del sensor!");
  } else {
    Serial.printf("INFO: presión %lf psi temperatura %lf C\r\n", pressure, temperature);
  }
}
