#ifndef MiLD2410_h
#define MiLD2410_h

#include "Arduino.h"
#include "HardwareSerial.h" // Necesario para la comunicación serial

// Definiciones de los comandos y respuestas del LD2410C (según datasheet)
// Estos son solo algunos ejemplos, la librería real manejaría más.
#define LD2410_HEADER1 0xFD
#define LD2410_HEADER2 0xFC
#define LD2410_HEADER3 0xFB
#define LD2410_HEADER4 0xFA
#define LD2410_TAIL1   0x04
#define LD2410_TAIL2   0x03
#define LD2410_TAIL3   0x02
#define LD2410_TAIL4   0x01

// Estructura para almacenar los datos del radar
struct LD2410_Data {
  bool hasMovingTarget;
  bool hasStationaryTarget;
  int movingDistance;     // Distancia del objetivo en movimiento en cm
  int stationaryDistance; // Distancia del objetivo estacionario en cm
  int movingEnergy;       // Energía del objetivo en movimiento
  int stationaryEnergy;   // Energía del objetivo estacionario
  // Puedes añadir más campos si necesitas los datos de los 9 gates, etc.
};

class MiLD2410 {
  public:
    // Constructor: Recibe un puntero a un objeto HardwareSerial
    MiLD2410(HardwareSerial* serialPort);

    // Método de inicialización
    void begin(long baudRate = 256000);

    // Método para leer datos del radar
    bool read();

    // Métodos para obtener el estado y la distancia
    bool isMoving();
    bool isStationary();
    int getMovingDistance();
    int getStationaryDistance();
    int getMovingEnergy();
    int getStationaryEnergy();

    // Métodos para configuración básica (puedes expandirlos)
    bool enterConfigMode();
    bool exitConfigMode();
    // bool setMaxGates(int movingGate, int stationaryGate); // Ejemplo de una función de configuración

  private:
    HardwareSerial* _serial;
    LD2410_Data _radarData;

    // Buffer para leer los datos del serial
    static const int BUFFER_SIZE = 64; // Tamaño suficiente para los paquetes de datos
    byte _serialBuffer[BUFFER_SIZE];
    int _bufferIndex;

    // Función auxiliar para buscar el encabezado de un paquete
    bool findHeader();
    // Función auxiliar para verificar el checksum (CRC) del paquete
    bool verifyChecksum(byte* buffer, int len);
    // Función auxiliar para parsear los datos del paquete
    void parseData(byte* buffer, int len);
};

#endif
