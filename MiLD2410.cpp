#include "MiLD2410.h"

MiLD2410::MiLD2410(HardwareSerial* serialPort) {
  _serial = serialPort;
  _bufferIndex = 0;
  // Inicializa los datos del radar a un estado conocido
  _radarData.hasMovingTarget = false;
  _radarData.hasStationaryTarget = false;
  _radarData.movingDistance = 0;
  _radarData.stationaryDistance = 0;
  _radarData.movingEnergy = 0;
  _radarData.stationaryEnergy = 0;
}

void MiLD2410::begin(long baudRate) {
  _serial->begin(baudRate);
  // Esperar un poco para que el módulo se inicie
  delay(100);
}

// Función para buscar el encabezado de un paquete
bool MiLD2410::findHeader() {
  while (_serial->available()) {
    byte incomingByte = _serial->read();
    // Si encontramos el primer byte del encabezado, comenzamos a buscar los siguientes
    if (incomingByte == LD2410_HEADER1) {
      if (_serial->read() == LD2410_HEADER2 &&
          _serial->read() == LD2410_HEADER3 &&
          _serial->read() == LD2410_HEADER4) {
        return true; // Encabezado encontrado
      }
    }
  }
  return false; // Encabezado no encontrado
}

// Función para verificar el checksum (simplificado, la implementación real es más compleja)
// NOTA: La verificación del checksum del LD2410 requiere el cálculo de un CRC16.
// Para este ejemplo básico, la omitiremos, pero es CRÍTICO para un sistema robusto.
// Deberías implementar un cálculo de CRC16 basado en el protocolo del LD2410.
bool MiLD2410::verifyChecksum(byte* buffer, int len) {
  // Aquí iría la lógica del CRC16. Para un ejemplo funcional, por ahora,
  // simplemente devolveremos true, asumiendo que los datos son correctos.
  // Esto es PELIGROSO en un sistema real.
  return true;
}

// Función para parsear los datos del paquete
void MiLD2410::parseData(byte* buffer, int len) {
  // El formato del paquete de datos es complejo y varía.
  // Aquí se asume un paquete de "frame de información de detección de destino".
  // Necesitarás la hoja de datos para mapear correctamente los bytes a los datos.
  // Este es un EJEMPLO basado en cómo se suelen estructurar estos módulos.

  // Byte 4 y 5: Longitud del payload (poco endian)
  uint16_t payloadLen = (uint16_t)buffer[5] << 8 | buffer[4];

  // Byte 6 (0x01) -> Estado del objetivo
  // Bit 0: Has Moving Target (objetivo en movimiento)
  // Bit 1: Has Stationary Target (objetivo estacionario)
  byte targetStatus = buffer[6];
  _radarData.hasMovingTarget = (targetStatus & 0b00000001) != 0;
  _radarData.hasStationaryTarget = (targetStatus & 0b00000010) != 0;

  // Los siguientes bytes son la distancia y energía
  // El formato exacto depende del "modo" de salida del módulo (básico o ingeniería)
  // y la versión de firmware.

  // EJEMPLO de cómo podrían estar los datos (ajusta según tu datasheet/pruebas):
  // Si el modo es básico (0xF001) y la longitud es 8 o más bytes de payload
  if (payloadLen >= 8) { // Ejemplo, puede ser más o menos
    // Distancia objetivo en movimiento (cm) - Bytes 7 y 8 (little endian)
    _radarData.movingDistance = (uint16_t)buffer[8] << 8 | buffer[7];
    // Energía objetivo en movimiento - Byte 9
    _radarData.movingEnergy = buffer[9];

    // Distancia objetivo estacionario (cm) - Bytes 10 y 11 (little endian)
    _radarData.stationaryDistance = (uint16_t)buffer[11] << 8 | buffer[10];
    // Energía objetivo estacionario - Byte 12
    _radarData.stationaryEnergy = buffer[12];
  } else {
      // Si la longitud del payload es menor de lo esperado, resetea los valores
      _radarData.movingDistance = 0;
      _radarData.stationaryDistance = 0;
      _radarData.movingEnergy = 0;
      _radarData.stationaryEnergy = 0;
  }
}

bool MiLD2410::read() {
  if (findHeader()) {
    // Si encontramos el encabezado, leemos la longitud del payload
    if (_serial->available() >= 2) { // Necesitamos al menos 2 bytes para la longitud del payload
      byte lenL = _serial->read();
      byte lenH = _serial->read();
      int payloadLength = (int)lenH << 8 | lenL; // Longitud total del payload

      // Asegurarse de que el buffer sea lo suficientemente grande
      if (payloadLength + 4 > BUFFER_SIZE) { // +4 para los 4 bytes de cola
        // Serial.println("Error: Payload demasiado grande para el buffer.");
        // Deberías manejar este error o aumentar BUFFER_SIZE
        while (_serial->available()) _serial->read(); // Limpiar el buffer serial
        return false;
      }

      // Leemos el resto del paquete (payload + 4 bytes de cola)
      int bytesToRead = payloadLength + 4; // payload + checksum(2) + end_bytes(2)
      if (_serial->available() >= bytesToRead) {
        // Almacenamos el encabezado que ya leímos para la verificación completa si es necesario
        _serialBuffer[0] = LD2410_HEADER1;
        _serialBuffer[1] = LD2410_HEADER2;
        _serialBuffer[2] = LD2410_HEADER3;
        _serialBuffer[3] = LD2410_HEADER4;
        _serialBuffer[4] = lenL;
        _serialBuffer[5] = lenH;

        for (int i = 6; i < 6 + bytesToRead; i++) {
          _serialBuffer[i] = _serial->read();
        }

        // Verifica el checksum (importante para datos fiables)
        // Por ahora, simplemente procesaremos. IMPLEMENTAR CRC16 REAL AQUÍ.
        if (verifyChecksum(_serialBuffer, 6 + bytesToRead)) {
          // Asegúrate de que los bytes finales sean correctos (0x04 0x03 0x02 0x01)
          if (_serialBuffer[6 + bytesToRead - 4] == LD2410_TAIL1 &&
              _serialBuffer[6 + bytesToRead - 3] == LD2410_TAIL2 &&
              _serialBuffer[6 + bytesToRead - 2] == LD2410_TAIL3 &&
              _serialBuffer[6 + bytesToRead - 1] == LD2410_TAIL4) {

            parseData(_serialBuffer, 6 + bytesToRead); // Parsear el paquete completo
            return true;
          }
        }
      }
    }
  }
  return false; // No se pudo leer un paquete válido
}

bool MiLD2410::isMoving() {
  return _radarData.hasMovingTarget;
}

bool MiLD2410::isStationary() {
  return _radarData.hasStationaryTarget;
}

int MiLD2410::getMovingDistance() {
  return _radarData.movingDistance;
}

int MiLD2410::getStationaryDistance() {
  return _radarData.stationaryDistance;
}

int MiLD2410::getMovingEnergy() {
  return _radarData.movingEnergy;
}

int MiLD2410::MiLD2410::getStationaryEnergy() {
  return _radarData.stationaryEnergy;
}


// --- Implementación de Funciones de Configuración (Ejemplo) ---
// Estas funciones envían comandos al radar. Necesitas los comandos exactos
// de la hoja de datos para cada operación.

// Ejemplo de comando "Entrar en modo configuración":
// FD FC FB FA 04 00 FF 00 01 00 (CRC2 bytes) 04 03 02 01
// Longitud del payload = 4 bytes (FF 00 01 00)
bool MiLD2410::enterConfigMode() {
  byte command[] = {
    LD2410_HEADER1, LD2410_HEADER2, LD2410_HEADER3, LD2410_HEADER4,
    0x04, 0x00, // Longitud del payload (4 bytes)
    0xFF, 0x00, 0x01, 0x00, // Payload (comando de configuración)
    0x00, 0x00, // Placeholder para CRC16 (calculado en tiempo real)
    LD2410_TAIL1, LD2410_TAIL2, LD2410_TAIL3, LD2410_TAIL4
  };
  // Aquí deberías calcular el CRC16 de los bytes desde el header hasta el final del payload
  // y reemplazar los 0x00, 0x00 del CRC.
  // _serial->write(command, sizeof(command));
  // Y luego leer la respuesta ACK del módulo.
  // Por simplicidad, solo imprimo el comando aquí.
  Serial.println("Comando enterConfigMode enviado (implementar envío real y CRC)");
  return true; // Asume éxito por ahora
}

// Ejemplo de comando "Salir de modo configuración":
// FD FC FB FA 02 00 FE 00 (CRC2 bytes) 04 03 02 01
// Longitud del payload = 2 bytes (FE 00)
bool MiLD2410::exitConfigMode() {
  byte command[] = {
    LD2410_HEADER1, LD2410_HEADER2, LD2410_HEADER3, LD2410_HEADER4,
    0x02, 0x00, // Longitud del payload (2 bytes)
    0xFE, 0x00, // Payload (comando para salir)
    0x00, 0x00, // Placeholder para CRC16
    LD2410_TAIL1, LD2410_TAIL2, LD2410_TAIL3, LD2410_TAIL4
  };
  // Calcular y enviar el comando.
  Serial.println("Comando exitConfigMode enviado (implementar envío real y CRC)");
  return true; // Asume éxito por ahora
}
