#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <RBDdimmer.h>

// Dirección del Multiplexor I2C
#define TCA9548A_ADDR 0x70

// Crear instancias de los sensores
Adafruit_SHT31 sht31_1 = Adafruit_SHT31();
Adafruit_SHT31 sht31_2 = Adafruit_SHT31();
Adafruit_SHT31 sht31_3 = Adafruit_SHT31();
Adafruit_SHT31 sht31_4 = Adafruit_SHT31();
Adafruit_SHT31 sht31_5 = Adafruit_SHT31();

// Pines para los colores del LED RGB
const int redPin = 11;    // Pin PWM para el color rojo
const int greenPin = 5;   // Pin PWM para el color verde
const int bluePin = 6;    // Pin PWM para el color azul
// Función para seleccionar el canal en el Multiplexor
void tca_select(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Configuración del dimmer
const int zeroCrossPin = 2;
const int acdPin = 3;
const int MIN_POWER = 0;
const int MAX_POWER = 90; // Set max power to 90%
dimmerLamp acd(acdPin);

// Configuración del relé
const int relayPin = 7; // Pin para el relé de control de temperatura
const int pinFan = 10;   // Pin del ventilador adicional para reducir la humedad

// Configuración de control de humedad
const int pinHumidificador = 8; // Pin del relé para el humidificador
const int pinExtractor = 9;     // Pin del relé para el extractor de aire
float setpointHum = 50; // Setpoint de humedad inicial (en %)

// Variables PID para temperatura
float Kc = 180;
float Tao_I = 60;
float Setpointem = 0.0;
float Setpointem1;
float PID_error = 0;
float Error_INT = 0;
float Potencia = 0;

// Variables para manejo del tiempo
unsigned long previousMillis = 0;
const long interval = 5000; // Intervalo de 5 segundos

void setup() {
  // Inicialización del puerto serial
  Serial.begin(9600);
  Serial.println(F("Sensor1\tSensor2\tSensor3\tSensor4\tSensor5\tSensorHum\tPromTemp\tPromHum\tSetTemp\tSetHum\tPotencia"));

  Wire.begin(); // Inicializar el bus I2C

  // Inicializar los sensores uno por uno seleccionando el canal en el multiplexor
  tca_select(0); // Seleccionar canal 0
  if (!sht31_1.begin(0x44)) {
    Serial.println("No se pudo encontrar el sensor SHT31_1!");
    while (1);
  }

  tca_select(1); // Seleccionar canal 1
  if (!sht31_2.begin(0x44)) {
    Serial.println("No se pudo encontrar el sensor SHT31_2!");
    while (1);
  }

  tca_select(2); // Seleccionar canal 2
  if (!sht31_3.begin(0x44)) {
    Serial.println("No se pudo encontrar el sensor SHT31_3!");
    while (1);
  }

  tca_select(3); // Seleccionar canal 3
  if (!sht31_4.begin(0x44)) {
    Serial.println("No se pudo encontrar el sensor SHT31_4!");
    while (1);
  }

  tca_select(4); // Seleccionar canal 4
  if (!sht31_5.begin(0x44)) {
    Serial.println("No se pudo encontrar el sensor SHT31_5!");
    while (1);
  }

  // Inicialización del dimmer
  acd.begin(NORMAL_MODE, ON);

  // Inicialización de los relés
  pinMode(relayPin, OUTPUT);
  pinMode(pinHumidificador, OUTPUT);
  pinMode(pinExtractor, OUTPUT);
  pinMode(pinFan, OUTPUT);

  // Inicializar los relés apagados
  digitalWrite(relayPin, HIGH);
  digitalWrite(pinHumidificador, LOW);
  digitalWrite(pinExtractor, LOW);
  digitalWrite(pinFan, LOW);
}

void loop() {
  // Leer comando desde el monitor serial
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Eliminar espacios en blanco al inicio y final
    
    if (input.equalsIgnoreCase("on")) {
      // Encender el LED RGB (color verde)
      setColor(0, 0, 0);
      //Serial.println("LED encendido");
    } else if (input.equalsIgnoreCase("off")) {
      // Apagar el LED RGB
      setColor(255, 255, 255);
      //Serial.println("LED apagado");
    } else {
      // Procesar otros comandos como setpoints
      char prefix = input.charAt(0);  // Leer el primer carácter como prefijo
      float value = input.substring(1).toFloat();  // Convertir el resto en número

      if (prefix == 'T') {
        Setpointem1 = value;
        Setpointem = value + 0.06;
        Serial.print("Setpoint de temperatura configurado a: ");
        Serial.print(Setpointem1);
        Serial.println(" °C");
      } else if (prefix == 'H') {
        setpointHum = value;
        Serial.print("Setpoint de humedad configurado a: ");
        Serial.print(setpointHum);
        Serial.println(" %");
      }
    }
  }

  // Control del dimmer siempre activo
  controlDimmer();

  // Leer y mostrar la temperatura y humedad a intervalos de 5 segundos
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Leer la temperatura y la humedad de los sensores
    tca_select(0);
    float temp1 = sht31_1.readTemperature();
    float hum1 = sht31_1.readHumidity();

    tca_select(1);
    float temp2 = sht31_2.readTemperature();
    float hum2 = sht31_2.readHumidity();

    tca_select(2);
    float temp3 = sht31_3.readTemperature();
    float hum3 = sht31_3.readHumidity();

    tca_select(3);
    float temp4 = sht31_4.readTemperature();
    float hum4 = sht31_4.readHumidity();

    tca_select(4);
    float temp5 = sht31_5.readTemperature();
    float hum5 = sht31_5.readHumidity();

    // Verificar que las lecturas no sean NaN
    if (isnan(temp1) || isnan(hum1) || isnan(temp2) || isnan(hum2) || 
        isnan(temp3) || isnan(hum3) || isnan(temp4) || isnan(hum4) ||
        isnan(temp5) || isnan(hum5)) {
      Serial.println("Error leyendo sensores");
    } else {
      // Calcular los promedios de temperatura y humedad entre sensores 3, 4 y 5
      float promTemp = (temp2 + temp3 + temp4 + temp5) / 4.0;
      float promHum = (hum2 + hum3 + hum4 + hum5) / 4.0;

      // Mostrar los valores de todos los sensores
      Serial.print(temp1); Serial.print("\t");
      Serial.print(temp2); Serial.print("\t");
      Serial.print(temp3); Serial.print("\t");
      Serial.print(temp4); Serial.print("\t");
      Serial.print(temp5); Serial.print("\t");
      Serial.print(hum1); Serial.print("\t");
      // Mostrar los promedios de temperatura y humedad
      Serial.print(promTemp); Serial.print("\t");
      Serial.print(promHum); Serial.print("\t");

      // Mostrar los setpoints actuales
      Serial.print(Setpointem1); Serial.print("\t");
      Serial.print(setpointHum); Serial.print("\t");

      // Calcular el valor PID para temperatura usando el promedio de temp3, temp4 y temp5
      calculatePID(promTemp);

      // Controlar los relés de humidificador y extractor de aire
      controlHumedad(promHum);
      // Controlar el ventilador de temperatura adicional
      controlTemperatura(promTemp);
      controlHumedadEx(promHum);
      // Mostrar el nivel de potencia actual
      Serial.print(Potencia);
      Serial.println("%");
    }
  }
}

void controlDimmer() {
  // Establecer la potencia calculada por el PID
  acd.setPower(Potencia);
}

void calculatePID(float temperature) {
  // Calcular el error
  PID_error = Setpointem - temperature;

  // Calcular la integral del error
  Error_INT += PID_error * (millis() - previousMillis) / 1000.0;

  // Calcular la salida del PID
  float PID_value = Kc * PID_error + (1.0 / Tao_I) * Error_INT;

  // Limitar la salida del PID
  if (PID_value < 0) {
    PID_value = 0;
  }
  if (PID_value > 100) {
    PID_value = 100;
  }

  // Asignar la salida del PID a la potencia
  Potencia = map(PID_value, 0, 100, MIN_POWER, MAX_POWER);
}

void controlTemperatura(float temperatura) {
  // Control del ventilador adicional para enfriar si la temperatura excede el setpoint
  if (temperatura > Setpointem1 + 0.20) {
    digitalWrite(relayPin, LOW); // Encender el ventilador (relé normalmente abierto)
  } else if (temperatura <= (Setpointem1 + 0.15)) {
    digitalWrite(relayPin, HIGH); // Apagar el ventilador
  }
}

/*void controlHumedad(float humedad) {
  // Control de la humedad con histéresis
  if (humedad < (setpointHum)) {
    // Si la humedad es menor que el setpoint, encender el humidificador y apagar el extractor
    digitalWrite(pinHumidificador, LOW); // Encender el humidificador
    digitalWrite(pinExtractor, HIGH);    // Apagar el extractor
    digitalWrite(pinFan, LOW);           // Apagar el ventilador adicional
  } else if (humedad > (setpointHum)) {
    // Si la humedad es mayor que el setpoint, apagar el humidificador y encender el extractor
    digitalWrite(pinHumidificador, HIGH); // Apagar el humidificador
    digitalWrite(pinExtractor, LOW);      // Encender el extractor
    digitalWrite(pinFan, HIGH);           // Encender el ventilador adicional
  }
}
*/
void controlHumedad(float humedad) {

  float hysteresis = 0.1;         // Margen de histéresis en %

  // Control del humidificador
  if (humedad < (setpointHum - hysteresis)) {
    // Activar humidificador cuando la humedad esté significativamente por debajo del setpoint
    digitalWrite(pinHumidificador, LOW); // Encender humidificador
    digitalWrite(pinFan, LOW);
    
  } else if (humedad >= (setpointHum - hysteresis)) {
    // Apagar humidificador cuando alcance o supere el setpoint
    digitalWrite(pinHumidificador, HIGH); // Apagar humidificador
    digitalWrite(pinFan, HIGH);
    
  }

}

void controlHumedadEx (float humedad){
  
  if (humedad > (setpointHum + 0.15)) {
    // Activar extractor cuando la humedad esté significativamente por encima del setpoint
    digitalWrite(pinExtractor, LOW); // Encender extractor
    
  } else if (humedad <= setpointHum + 0.1) {
    // Apagar extractor cuando la humedad esté dentro del rango aceptable
    digitalWrite(pinExtractor, HIGH); // Apagar extractor
    
  }
}

// Función para establecer el color del LED RGB
void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}