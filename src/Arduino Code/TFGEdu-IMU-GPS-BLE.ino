//IMPORTAMOS LAS LIBRERIAS NECESARIAS:
#include "MadgwickAHRS.h" //Librería necesaria para realizar el cálculo de quaterniones para determinar la orientación de la pala.
#include "TinyGPS.h" //Librería necesaria para realizar el manejo de las trazas NMEA y conseguir el cálculo de latitud y longitud, entre otros.
#include <ArduinoBLE.h> //Librería necesaria para construir la infraestructura bluetooth con todos los servicios y características necesarias.
#include <Arduino_LSM9DS1.h> //Librería necesaira para acceder a las lecturas del sensor IMU LSM9DS1.

//Declaración de variables globales, servicios y características.
TinyGPS gps;
Madgwick orientation;

const float gyracc_rate = 119.00; //Frecuencia de muestreo máxima del acelerómetro y giroscopio. (ref: https://reference.arduino.cc/reference/en/libraries/arduino_lsm9ds1/) 
static void smartdelay(unsigned long ms); //Método que aplica un delay sin dejar de leer el puerto serial para no perder el hilo de ejecución.

//Se declaran los servicios IMUService y GPSService.
BLEService IMUService("0x023I");
BLEService GPSService("0x023G");

//Se definen las características del servicio IMUService.
BLECharacteristic AccelerometerData("0x023I001A", BLERead | BLENotify, 20);
BLECharacteristic GyroscopeData("0x023I002G", BLERead | BLENotify, 20);
BLECharacteristic MagnetometerData("0x023I003M", BLERead | BLENotify, 20);
BLECharacteristic OrientationData("0x023I004O", BLERead | BLENotify, 20);

//Se define la característica para el servicio GPSService.
BLECharacteristic GPSData("0x023G001D", BLERead | BLENotify, 100);

void setup() {

  Serial.begin(9600); //Se inicializa el puerto Serial.
  Serial1.begin(9600); //Se inicializa el puerto Serial1 (puerto UART).

  if(!BLE.begin()) {
    //Si el módulo BLE no ha iniciado correctamente se ejecuta el siguiente comando:
    while(true); //Espera infinitamente
  }

  if(!IMU.begin()) {
    //Si el módulo IMU no ha iniciado correctamente, se ejecuta este comando:
    while(true);
  }

  //Nombre por el que se identificará al dispositivo.
  BLE.setDeviceName("33BLE");
  BLE.setLocalName("33BLE");	

  BLE.setAdvertisedService(IMUService);	//Se establece el nombre del servicio IMUService para ser difundido.
  BLE.setAdvertisedService(GPSService); //Se establece el nombre del servicio GPSService para ser difundido.

  //Se agregan las diferentes características al servicio IMUService y se añade el servicio a la pila BLE.
  IMUService.addCharacteristic(AccelerometerData);
  IMUService.addCharacteristic(GyroscopeData);
  IMUService.addCharacteristic(MagnetometerData);
  IMUService.addCharacteristic(OrientationData);
  BLE.addService(IMUService);

  //Se agregan las diferentes características al servicio GPSService y se añade el servicio a la pila BLE.
  GPSService.addCharacteristic(GPSData);
  BLE.addService(GPSService);
  
  BLE.setConnectionInterval(0.0075, 4); //Establecemos un intérvalo de conexión: el mínimo es de 7.5ms y el máximo es de 4s (ref: https://punchthrough.com/maximizing-ble-throughput-on-ios-and-android/) 
  BLE.setConnectable(true); //Se habilita el periférico para que otros dispositivos puedan conectarse
  BLE.advertise(); //Se inicia la difusion de los servicios.

  orientation.begin(gyracc_rate); //Se inicia el filtro de madgwick para el cálculo de la orientación.
}

void loop() {
  //Se declaran las variables que almacenarán la información capturada por el sensor IMU
  float xAcc, yAcc, zAcc;
  float xGyro, yGyro, zGyro;
  float xMag, yMag, zMag;

  //Se declaran las variables que almacenarán la información capturada por el sensor GPS.
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;

  BLEDevice central = BLE.central(); //Se crea un objeto central que se conectará a la transmisión de este periférico y recibirá sus datos.

  if (central) {
    // Si se conecta un central a este periférico:
    while (central.connected()) {
      //Mientras que el central esté escuchando la transmisión:
      if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() && IMU.magneticFieldAvailable()) {
        //Si los datos del sensor IMU están disponibles:
        
        IMU.readAcceleration(xAcc, yAcc, zAcc); //Lectura de los datos del acelerómetro y asignación a las variables [xAcc, yAcc, zAcc] respectivamente.
        String accString = String(xAcc)+","+String(yAcc)+","+String(zAcc); //Se parsean los datos a strings para poder enviar por BLE sin problemas de conversión.
        AccelerometerData.writeValue(accString.c_str()); //Se escribe el valor de la lectura en la característica correspondiente al acelerómetro.

        IMU.readGyroscope(xGyro, yGyro, zGyro); //Lectura de los datos del giroscopio y asignación a las variables [xGyro, yGyro, zGyro] respectivamente.
        String gyroString = String(xGyro)+","+String(yGyro)+","+String(zGyro); //Se parsean los datos a strings para poder enviar por BLE sin problemas de conversión.
        GyroscopeData.writeValue(gyroString.c_str()); //Se escribe el valor de la lectura en la característica correspondiente al giroscopio.

        IMU.readMagneticField(xMag, yMag, zMag); //Lectura de los datos del magnetómetro y asignación a las variables [xMag, yMag, zMag] respectivamente.
        String magString = String(xMag)+","+String(yMag)+","+String(zMag); //Se parsean los datos a strings para poder enviar por BLE sin problemas de conversión.
        MagnetometerData.writeValue(magString.c_str()); //Se escribe el valor de la lectura en la característica correspondiente al magnetómetro.

        orientation.update(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc, xMag, yMag, zMag); //Actualización de los datos de orientación y asignación a las variables [roll, pitch, yaw] respectivamente.
        String orientString = String(orientation.getRoll())+","+String(orientation.getPitch())+","+String(orientation.getYaw()); //Se parsean los datos a strings para poder enviar por BLE sin problemas de conversión.
        OrientationData.writeValue(orientString.c_str()); //Se escriben los valores en la característica correspondiente a la orientación.

        smartdelay(10); //Se deja un delay de 10ms para actualizar el buffer de nuevos valores.
      }
      //Comprobación de que los datos que vienen del GPS sean correctos.
      if (gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES){ //Si falla la conexión satelital, evitamos enviar la traza GPS.
        Serial.println("¡Se ha realizado la conexión satelital con éxito!"); //Mostrar por pantalla si se ha realizado la conexión con satélites.
        String satellites = String(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites()); //Obtención el número de satélites conectados y parsamos a String.
        String hdop = String(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop()); //Obtención de la dilución horizontal de la precisión.
        gps.f_get_position(&flat, &flon, &age); //Obtención de latitud, longitud y age y asignación a las variables [flat, flon, age] respectivamente.
        String latitude = String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6); //Parsing a String.
        String longitude = String(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6); //Parsing a String.

        gps.stats(&chars, &sentences, &failed); //Obtención de parámetros de la transmisión.
        String gpsString = satellites+","+hdop+","+latitude+","+longitude+","+chars+","+sentences+","+failed; //Generación de la traza completa GPS.
        GPSData.writeValue(gpsString.c_str()); //Escritura de datos en la característica GPSData para el servicio GPSService.
        
        //Log para trazabilidad
        Serial.println("Enviada traza por bluetooth:"); 
        Serial.println(gpsString); //
        
        smartdelay(1000); //Se crea un delay de 1 segundo para actualizar el buffer del puerto Serial1 al 100%.
      }
      else {
        Serial.println("Todavía no se ha conectado a ningún satélite.");
        smartdelay(1000);
      }
    }
  }
}

static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}
