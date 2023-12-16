#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include <Preferences.h>

#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHT.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <HardwareSerial.h>

HardwareSerial SerialToArduino(1); // Uso de Serial1 en el ESP32
//pines de sensores
#define DHTPIN  21  // Pin digital al que está conectado el DHT11
#define DHTTYPE DHT11           // Tipo de sensor DHT utilizado
#define motionSensorPin   4 // Pin al que está conectado el sensor de movimiento
#define soundSensorPin    2 // Pin analógico al que está conectado el sensor de sonido
#define mq2Pin            5 // Pin analógico al que está conectado el detector MQ2
SoftwareSerial ss       (22, 23); //TX, RX pin para gps
//pines de actuadores
#define buzzerPin     13
DHT dht(DHTPIN, DHTTYPE);
#define AIO_USERNAME    "PawSecure"
#define AIO_KEY         "aio_PRAR57H63I99IiCg0z4fVsVXz09y"
// Variables para almacenar el nombre y la contraseña de la red WiFi
String ssid;
String password;
String id;
//instanciar bluetooth, preferencias, wifi cliente, gps
BluetoothSerial SerialBT;
Preferences preferences;
WiFiClient client;
TinyGPS gps;
char command[3]; //para comandos_bluetooth
//variables sensores
int   motionValue = 0;
int   soundValue  = 0;
int   mq2Value    = 0;
float humidity    = 0;
float temperature = 0;
int iteracion     = 0;
String gps_value ;
String gps_value_pasado;
bool actor_temperatura =true;
float X_general   = 0;
float Y_general   = 0;
float Z_general   = 0;
float XYZ_ANTERIOR = 0;
float XYZ_NUEVA    = 0;
int numero=0;

//gps
float flat, flon;
unsigned long age;
unsigned short sentences = 0, failed = 0;
char latBuffer[10]; // Buffer para almacenar la cadena de latitud
char lonBuffer[10]; // Buffer para almacenar la cadena de longitud
char latBuffer_pasado[10];
char lonBuffer_pasado[10];

//variables de tiempo, y otras 
unsigned long previousMillis = 0; //
unsigned long previousMillis_2 = 0; //
unsigned long previousMillis_mq2 = 0;
unsigned long previousMillis_movimiento = 0;
unsigned long previousMillis_ = 0;
const long interval = 15000;  // Intervalo de 5 segundos
const long interval_enviar = 120000;
unsigned long currentMillis = 0;  // Variable para almacenar el tiempo actual
unsigned long tiempo_inicio_movimiento=0;
int q = 0; // para la contraseña y clave de bluetooth 
int i = 0;
bool luces_encendida      = true;
bool ventilador_encendido = true ;
bool buzzer_encendido     = true;

//feed
Adafruit_MQTT_Client mqtt(&client, "io.adafruit.com", 1883, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish feed_humedad      = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humedad ");
Adafruit_MQTT_Publish feed_temperatura  = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatura");
Adafruit_MQTT_Publish feed_movimiento   = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/movimiento");
Adafruit_MQTT_Publish feed_sonido       = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sonido");
Adafruit_MQTT_Publish feed_gps          = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gps");
Adafruit_MQTT_Publish feed_gas          = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/mq2");
Adafruit_MQTT_Publish feed_acelerometro = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/acelerometro");
//acelerometro 
#define ADXL345_ADDRESS (0x53)

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(ADXL345_ADDRESS);
sensors_event_t baseValues;  // Almacena los valores promedio en reposo
sensors_event_t gravityValues;  // Almacena los valores promedio en gravedad
sensors_event_t event;


void setup() {
  Serial.begin(9600);// Inicializa el puerto serie
  SerialBT.begin("");// Inicializa el Bluetooth
  ss.begin(9600);//inicializar gps
  Wire.begin(18, 19);//acelerometro
  SerialToArduino.begin(9600);
 
  delay(5000);
  if (!accel.begin()) {
    Serial.println("No se pudo iniciar el ADXL345. Por favor, revise las conexiones.");
  } else{
      acelerometro_calibracion();
  }
  tryLoadAndConnectToWifi();//intenta conectarse al wifi

  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // Actualiza el tiempo actual
  currentMillis = millis();
  procesar_datos_sensores();
  // Recibe los datos del Bluetooth solo si no está conectado a la red WiFi
  
  if (WiFi.status() != WL_CONNECTED) {

    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Credenciales nulas, cargando por bluetooth");
    while ( WiFi.status() != WL_CONNECTED) {
      if(SerialBT.available() ){
      char receivedChar = SerialBT.read();
      if(receivedChar=='{'){
      conectandoPorBluetooth();
      }
        actuadores_bluetooth(receivedChar);
        actuadores();
        procesar_datos_sensores();
      }
    }
  }
  else{
    mqtt.connect();
  if (mqtt.connected()) {
     if (currentMillis - previousMillis_2 >= interval_enviar) {
      procesar_y_enviar_datos_sensores2();
      previousMillis_2 = currentMillis;
     }
    }
  }
  actuadores();
      if(SerialBT.available()>0)
      {
        char receivedChar = SerialBT.read();
        actuadores_bluetooth(receivedChar);}
  delay(2000);
}

void actuadores_bluetooth(char receivedChar){
    Serial.print(receivedChar);
    Serial.println(); 


  if (receivedChar =='L'){//&& luces_encendida) {
      SerialToArduino.write("J");
      Serial.println("luces encendidas");
    }
  

  if (receivedChar == 'V' ){//&& ventilador_encendido) {
        SerialToArduino.write("X"); // Comando para ventilador
        Serial.println("ventilador encendidas");
      }


    if (receivedChar == 'B'){ //&& buzzer_encendido) {
      if (digitalRead(buzzerPin) == HIGH) {
        Serial.println("Comando recibido desde la app: Apagar buccer");
        digitalWrite(buzzerPin, LOW); // Apaga las luces
      } else {
        Serial.println("Comando recibido desde la app: Encender buccer");
        digitalWrite(buzzerPin, HIGH); // Enciende las luces
      }
    }
}

//voids de wifi y bluetooth
static void processData() {

  previousMillis = millis();
  
  Serial.print("nombre:");
  Serial.println(ssid);
  Serial.print("contraseña:");
  Serial.println(password);
  Serial.print("id:");
  Serial.println(id);

  // Conecta a la red WiFi
  Serial.println("Conectando a la red WiFi...");
  WiFi.begin(ssid.c_str(), password.c_str());

  // Espera a que la conexión se establezca

  while(millis() - previousMillis<= interval && WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  // La conexión se estableció
  if (WiFi.status() == WL_CONNECTED) {
    // Guardar/reemplazar namespace
    preferences.begin("credenciales", false);
    preferences.putString("ssid", ssid);
    preferences.putString("password", password);
    preferences.putString("id", id);
    preferences.end();
    Serial.println("Se guardaron las credenciales.\n");
    Serial.println("");
    Serial.println("Conectado a la red WiFi.");
    while(millis() - previousMillis<= interval && WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  } else {
    Serial.print("");
    Serial.println("Nombre o contraseña incorrectos, intente de nuevo");
    ssid = "";
    password = "";
    id="";
    q=0; 
  }
}


static void conectandoPorBluetooth() {
  // Lee un byte del Bluetooth
  char c;
  do {
    if (SerialBT.available()) {
      c = SerialBT.read();
      Serial.println(c);

      if (c == ':') {
        q += 1;
      } else {
        if (q == 0 && c != '}') {
          ssid += c;
        } else if(q == 1 && c != '}') {
          password += c;
        } else if(q == 2 && c != '}') {
          id += c;
        }
      }
    }
  } while (c != '}');
    processData();
}

static void tryLoadAndConnectToWifi() {
  previousMillis = millis();
  preferences.begin("credenciales", false);
  ssid     = preferences.getString("ssid", "");
  password = preferences.getString("password", "");
  id       = preferences.getString("id", "");
  if (ssid != "" && password != "" && id !="") {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.print("Credenciales cargadas de memoria. \nConectando al WiFi...");
    while(millis() - previousMillis<= interval && WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  } 
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void print_float(float val, float invalid, int len, int prec)
{
  if (val != invalid)
  {
    //Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
  }
  smartdelay(0);
}
//void de gps















//procesar datos 
void procesar_datos_sensores(){

    float flat, flon;
    unsigned long age;
    unsigned short sentences = 0, failed = 0;

    gps.f_get_position(&flat, &flon, &age);
    print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 6);
    Serial.println();
    smartdelay(10000);
    
  sensors_event_t event;
  accel.getEvent(&event);

  // Aplicar ajustes de calibración
  event.acceleration.x -= baseValues.acceleration.x;
  event.acceleration.y -= baseValues.acceleration.y;
  event.acceleration.z -= baseValues.acceleration.z;

    motionValue += digitalRead(motionSensorPin);
    soundValue  += digitalRead(soundSensorPin);
  if(!digitalRead(mq2Pin))
    {
    mq2Value += 1;
    }

    humidity    += dht.readHumidity(DHTPIN);
    temperature += dht.readTemperature(DHTPIN);
    X_general   += abs(event.acceleration.x);
    Y_general   += abs(event.acceleration.y);
    Z_general   += abs(event.acceleration.z);

    iteracion   += 1;

    Serial.println("X: " + String(event.acceleration.x) + ", Y: " + String(event.acceleration.y) + ", Z: " + String(event.acceleration.z));
    Serial.println(digitalRead(motionSensorPin));
    Serial.println(digitalRead(mq2Pin));
    Serial.println(dht.readHumidity());
    Serial.println(dht.readTemperature());
    Serial.println(String(flat)+String(flon));

}

void procesar_y_enviar_datos_sensores2(){

    dtostrf(flat, 6, 5, latBuffer);
    dtostrf(flon, 7, 5, lonBuffer);

    gps_value = "latitud: " + String(latBuffer) + "/longitud: " + String(lonBuffer);
    //feed

    if(motionValue >0){feed_movimiento.publish  (("U"+ id +":1").c_str());}

    if(soundValue >0) {feed_sonido.publish      (("U"+ id +":1").c_str());}

    if(mq2Value >0)   {feed_gas.publish         (("U"+ id +":1").c_str());}

    XYZ_NUEVA=((X_general+Y_general+Z_general)/iteracion);

if(XYZ_NUEVA>XYZ_ANTERIOR+.3){// si sube mas del 30 
feed_acelerometro.publish(String("U" + String(id) + ":X" + String(X_general / iteracion) + "Y" + String(Y_general / iteracion) + "Z" + String(Z_general / iteracion)).c_str());
  XYZ_ANTERIOR=XYZ_NUEVA;
}
    if (latBuffer != latBuffer_pasado && lonBuffer != lonBuffer_pasado) {
      feed_gps.publish((gps_value).c_str());
    strcpy(latBuffer_pasado, latBuffer);
    strcpy(lonBuffer_pasado, lonBuffer);
    }


    feed_humedad.publish      (enviar(humidity).c_str());
    feed_temperatura.publish  (enviar(temperature).c_str());

    motionValue = 0;
    soundValue  = 0;
    mq2Value    = 0;

    humidity    = 0;
    temperature = 0;

    X_general   = 0;
    Y_general   = 0;
    Z_general   = 0;

    iteracion   = 0;
    //Serial.print(mensaje);
}
String enviar(float valor){
  String datos = "U" + id + ":" + String(valor / iteracion);
return datos; 
}










//actuadores 
void actuadores()// principales
{


  
  // Activar el buzzer y ventilador si se detectan niveles peligrosos de gas
  if (digitalRead(mq2Pin) == 0) //prendido
  {
    Serial.print(digitalRead(mq2Pin));
    digitalWrite(buzzerPin, HIGH);
    //ventiladorPin, HIGH);
    SerialToArduino.write('V');

    Serial.println("Buzzer y ventilador encendidos por el mq2");
    buzzer_encendido = false;
    ventilador_encendido =false;
    previousMillis_mq2 = millis(); // Reinicia el cronómetro
  }

  // Apagar buzzer y ventilador después de 30 segundos
  if (!buzzer_encendido && !ventilador_encendido && millis() - previousMillis_mq2 >= 10000) {
    digitalWrite(buzzerPin, LOW);
    SerialToArduino.write('C');
    Serial.println("Buzzer y ventilador apagados después de 30 segundos");
    buzzer_encendido = true;
    ventilador_encendido =true;

  }

  // Encender luces si se detecta movimiento
  if (digitalRead(motionSensorPin) == 1) {
    //digitalWrite(ledsPin, HIGH);
    //SerialToArduino.write('L');
    luces_encendida =false;
    Serial.println("Luces encendidas por el movimiento");
    previousMillis_movimiento = millis(); // Reinicia el cronómetro
  }

  // Apagar luces después de 30 segundos
  if (!luces_encendida && millis() - previousMillis_movimiento >= 10000) {

    //digitalWrite(ledsPin, LOW);
    SerialToArduino.write('K');
    Serial.println("Luces apagadas después de 30 segundos");
    luces_encendida = true; // Reinicia la variable
  }

}




//acelerometro 

 void acelerometro_calibracion(){
  accel.setRange(ADXL345_RANGE_2_G); // ±2g
  Serial.println("ADXL345 iniciado correctamente");
  delay(5000);  
  calibrateRest();
  calibrateGravity(); 
}

 void calibrateRest() {
  // Calibración en reposo
  int numSamples = 100;
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    baseValues.acceleration.x += event.acceleration.x;
    baseValues.acceleration.y += event.acceleration.y;
    baseValues.acceleration.z += event.acceleration.z;
    delay(50);
  }

  baseValues.acceleration.x /= numSamples;
  baseValues.acceleration.y /= numSamples;
  baseValues.acceleration.z /= numSamples;
}

void calibrateGravity() {
  // Calibración en gravedad
  int numSamples = 100;
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    gravityValues.acceleration.x += event.acceleration.x;
    gravityValues.acceleration.y += event.acceleration.y;
    gravityValues.acceleration.z += event.acceleration.z;
    delay(50);
  }

  gravityValues.acceleration.x /= numSamples;
  gravityValues.acceleration.y /= numSamples;
  gravityValues.acceleration.z /= numSamples;
}






