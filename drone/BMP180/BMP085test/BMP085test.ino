#include <Wire.h>
#include <Adafruit_BMP085.h>

// Nombre de mesures pour la moyenne (stabilisation)
const int NOMBRE_MESURES = 50;

// Pression au niveau de la mer par défaut (en hPa)
// Ajuste cette valeur selon la météo locale pour plus de précision
#define SEALEVELPRESSURE_HPA 1013.25

Adafruit_BMP085 bmp;
const int NOMBRE_MESURES_ref = 50;
float Altitude_ref = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("\n--- Initialisation du BMP180 sur ESP8266 ---");

  // Vérification de la présence du capteur
  if (!bmp.begin()) {
    Serial.println("Erreur : Impossible de trouver le capteur BMP180 !");
    Serial.println("Vérifiez le câblage (SDA -> D2, SCL -> D1, VCC -> 3V3, GND -> GND).");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("Capteur BMP180 détecté et initialisé avec succès !");

  for (int i = 0; i < NOMBRE_MESURES_ref; i++) {
    Altitude_ref += bmp.readAltitude();
    delay(5); // Pause de 50 ms entre chaque lecture
  }

  Altitude_ref = Altitude_ref/NOMBRE_MESURES_ref;

  Serial.print("Altitude de Référence :");
  Serial.print(Altitude_ref);
  
}

void loop() {
  float sommeAltitude = 0;
  // float sommePression = 0;

  // Prise de plusieurs mesures pour calculer la moyenne
  for (int i = 0; i < NOMBRE_MESURES; i++) {
    // sommePression += bmp.readPressure();
    sommeAltitude += bmp.readAltitude();
    delay(1); // Pause de 1 ms entre chaque lecture
  }

  // Calcul des moyennes
  // float pressionMoyennehPa = (sommePression / NOMBRE_MESURES) / 100.0;
  float altitudeMoyenneMetres = sommeAltitude / NOMBRE_MESURES;

  // Affichage des résultats stabilisés
  // Serial.println("===============================");
  // Serial.print("Pression moyenne : ");
  // Serial.print(pressionMoyennehPa, 2);
  // Serial.println(" hPa");

  float Altitude = altitudeMoyenneMetres - Altitude_ref;

  Serial.print("Altitude calculée : ");
  Serial.print(Altitude, 2);
  Serial.println(" mètres");
  Serial.println("===============================\n");

}