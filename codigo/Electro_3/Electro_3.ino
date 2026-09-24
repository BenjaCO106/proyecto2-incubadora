#include <Wire.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <MFRC522.h>

// =====================================================
// PINES ESP32
// =====================================================
#define PIN_I2C_SDA    21
#define PIN_I2C_SCL    22

#define PIN_ONEWIRE     4

#define PIN_RC522_CS    5
#define PIN_RC522_RST  27
#define PIN_SPI_SCK    18
#define PIN_SPI_MISO   19
#define PIN_SPI_MOSI   23

// =====================================================
// MPU6500
// =====================================================
#define MPU6500_ADDR          0x68
#define REG_WHO_AM_I          0x75
#define REG_PWR_MGMT_1        0x6B
#define REG_PWR_MGMT_2        0x6C
#define REG_SMPLRT_DIV        0x19
#define REG_CONFIG            0x1A
#define REG_GYRO_CONFIG       0x1B
#define REG_ACCEL_CONFIG      0x1C
#define REG_ACCEL_CONFIG_2    0x1D
#define REG_ACCEL_XOUT_H      0x3B

// =====================================================
// INTERVALOS
// =====================================================
const unsigned long INTERVALO_REPORTE_MS = 1000;
const unsigned long INTERVALO_MPU_MS = 100;
const unsigned long CONVERSION_DS18B20_MS = 750;

// =====================================================
// OBJETOS
// =====================================================
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensorDS18B20(&oneWire);

MAX30105 sensorMAX;

MFRC522 sensorRFID(PIN_RC522_CS, PIN_RC522_RST);

// =====================================================
// ESTADO DE LOS MÓDULOS
// =====================================================
bool statusDS18B20 = false;
bool statusMAX30102 = false;
bool statusMPU6500 = false;
bool statusRFID = false;

// =====================================================
// TEMPORIZACIÓN
// =====================================================
unsigned long tiempoReporteAnterior = 0;
unsigned long tiempoMPUAnterior = 0;
unsigned long tiempoSolicitudDS = 0;

// =====================================================
// VARIABLES DS18B20
// =====================================================
bool conversionDSActiva = false;
bool temperaturaValida = false;
float temperaturaCabina = 0.0;

// =====================================================
// VARIABLES MAX30102
// =====================================================
uint32_t valorRojo = 0;
uint32_t valorIR = 0;

const byte CANTIDAD_BPM = 4;
byte valoresBPM[CANTIDAD_BPM];

byte posicionBPM = 0;
byte cantidadBPMValidos = 0;

unsigned long tiempoUltimoLatido = 0;

float bpmInstantaneo = 0.0;
int bpmPromedio = 0;

// =====================================================
// VARIABLES MPU6500
// =====================================================
bool datosMPUValidos = false;

float aceleracionX = 0.0;
float aceleracionY = 0.0;
float aceleracionZ = 0.0;

float giroX = 0.0;
float giroY = 0.0;
float giroZ = 0.0;

// =====================================================
// DECLARACIÓN DE FUNCIONES
// =====================================================
bool escribirRegistroMPU(byte registro, byte valor);
byte leerRegistroMPU(byte registro);
bool leerBloqueMPU(byte registroInicial, byte *datos, byte cantidad);
bool inicializarMPU6500();

void verificarRFID();
void actualizarMAX30102();
void solicitarTemperaturaDS();
void actualizarDS18B20(unsigned long tiempoActual);
void actualizarMPU6500(unsigned long tiempoActual);
void mostrarTelemetria();

// =====================================================
// CONFIGURACIÓN
// =====================================================
void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" ESTACION NEONATAL - ESP32");
  Serial.println("==========================================");

  // Bus I2C para MAX30102 y MPU6500
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  // Bus SPI para RC522
  SPI.begin(
    PIN_SPI_SCK,
    PIN_SPI_MISO,
    PIN_SPI_MOSI,
    PIN_RC522_CS
  );

  // ---------------------------------------------------
  // DS18B20
  // ---------------------------------------------------
  sensorDS18B20.begin();

  if (sensorDS18B20.getDeviceCount() > 0) {
    statusDS18B20 = true;

    sensorDS18B20.setResolution(12);
    sensorDS18B20.setWaitForConversion(false);

    solicitarTemperaturaDS();

    Serial.println("[OK] DS18B20 detectado");
  } else {
    Serial.println("[ERROR] DS18B20 no detectado");
  }

  // ---------------------------------------------------
  // MAX30102
  // ---------------------------------------------------
  if (sensorMAX.begin(Wire, I2C_SPEED_FAST)) {
    statusMAX30102 = true;

    /*
      Intensidad LED: 0x1F
      Promedio: 4 muestras
      Modo: LED rojo + infrarrojo
      Frecuencia: 100 muestras/s
      Ancho de pulso: 411 us
      Rango ADC: 4096
    */
    sensorMAX.setup(
      0x1F,
      4,
      2,
      400,
      411,
      4096
    );

    sensorMAX.setPulseAmplitudeRed(0x0A);
    sensorMAX.setPulseAmplitudeIR(0x1F);
    sensorMAX.setPulseAmplitudeGreen(0);

    Serial.println("[OK] MAX30102 detectado en 0x57");
  } else {
    Serial.println("[ERROR] MAX30102 no detectado");
  }

  // ---------------------------------------------------
  // MPU6500
  // ---------------------------------------------------
  statusMPU6500 = inicializarMPU6500();

  if (statusMPU6500) {
    Serial.println("[OK] MPU6500 detectado en 0x68");
  } else {
    Serial.println("[ERROR] MPU6500 no detectado");
  }

  // ---------------------------------------------------
  // RFID-RC522
  // ---------------------------------------------------
  sensorRFID.PCD_Init();

  byte versionRC522 =
    sensorRFID.PCD_ReadRegister(sensorRFID.VersionReg);

  if (versionRC522 != 0x00 && versionRC522 != 0xFF) {
    statusRFID = true;

    Serial.print("[OK] RC522 detectado - Version: 0x");
    Serial.println(versionRC522, HEX);
  } else {
    Serial.println("[ERROR] RC522 no detectado");
  }

  Serial.println("==========================================");
  Serial.println(" INICIALIZACION FINALIZADA");
  Serial.println("==========================================");
}

// =====================================================
// BUCLE PRINCIPAL
// =====================================================
void loop() {
  unsigned long tiempoActual = millis();

  // RFID se comprueba continuamente
  verificarRFID();

  // Se procesan las muestras almacenadas por el MAX30102
  actualizarMAX30102();

  // Lectura asíncrona del DS18B20
  actualizarDS18B20(tiempoActual);

  // Lectura periódica del MPU6500
  actualizarMPU6500(tiempoActual);

  // Reporte general cada segundo
  if (tiempoActual - tiempoReporteAnterior
      >= INTERVALO_REPORTE_MS) {

    tiempoReporteAnterior = tiempoActual;
    mostrarTelemetria();
  }
}

// =====================================================
// FUNCIONES MPU6500
// =====================================================
bool escribirRegistroMPU(byte registro, byte valor) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(registro);
  Wire.write(valor);

  return Wire.endTransmission() == 0;
}

byte leerRegistroMPU(byte registro) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(registro);

  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }

  Wire.requestFrom(
    (uint8_t)MPU6500_ADDR,
    (uint8_t)1,
    true
  );

  if (Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

bool leerBloqueMPU(
  byte registroInicial,
  byte *datos,
  byte cantidad
) {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(registroInicial);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  byte recibidos = Wire.requestFrom(
    (uint8_t)MPU6500_ADDR,
    cantidad,
    true
  );

  if (recibidos != cantidad) {
    while (Wire.available()) {
      Wire.read();
    }

    return false;
  }

  for (byte i = 0; i < cantidad; i++) {
    datos[i] = Wire.read();
  }

  return true;
}

bool inicializarMPU6500() {
  byte identificacion = leerRegistroMPU(REG_WHO_AM_I);

  Serial.print("MPU WHO_AM_I: 0x");
  Serial.println(identificacion, HEX);

  if (identificacion != 0x70) {
    return false;
  }

  // Reinicio
  escribirRegistroMPU(REG_PWR_MGMT_1, 0x80);
  delay(100);

  // Activación y reloj interno
  escribirRegistroMPU(REG_PWR_MGMT_1, 0x01);
  escribirRegistroMPU(REG_PWR_MGMT_2, 0x00);

  // Frecuencia aproximada de 100 Hz
  escribirRegistroMPU(REG_SMPLRT_DIV, 0x09);

  // Filtro digital
  escribirRegistroMPU(REG_CONFIG, 0x04);

  // Giroscopio ±500 grados/s
  escribirRegistroMPU(REG_GYRO_CONFIG, 0x08);

  // Acelerómetro ±8 g
  escribirRegistroMPU(REG_ACCEL_CONFIG, 0x10);

  // Filtro del acelerómetro
  escribirRegistroMPU(REG_ACCEL_CONFIG_2, 0x04);

  return true;
}

// =====================================================
// ACTUALIZACIÓN MPU6500
// =====================================================
void actualizarMPU6500(unsigned long tiempoActual) {
  if (!statusMPU6500) {
    return;
  }

  if (tiempoActual - tiempoMPUAnterior
      < INTERVALO_MPU_MS) {

    return;
  }

  tiempoMPUAnterior = tiempoActual;

  byte datos[14];

  if (!leerBloqueMPU(
        REG_ACCEL_XOUT_H,
        datos,
        14
      )) {

    datosMPUValidos = false;
    return;
  }

  int16_t axRaw =
    (int16_t)((datos[0] << 8) | datos[1]);

  int16_t ayRaw =
    (int16_t)((datos[2] << 8) | datos[3]);

  int16_t azRaw =
    (int16_t)((datos[4] << 8) | datos[5]);

  int16_t gxRaw =
    (int16_t)((datos[8] << 8) | datos[9]);

  int16_t gyRaw =
    (int16_t)((datos[10] << 8) | datos[11]);

  int16_t gzRaw =
    (int16_t)((datos[12] << 8) | datos[13]);

  // Conversión para rango ±8 g
  aceleracionX = (axRaw / 4096.0) * 9.80665;
  aceleracionY = (ayRaw / 4096.0) * 9.80665;
  aceleracionZ = (azRaw / 4096.0) * 9.80665;

  // Conversión para rango ±500 grados/s
  giroX = gxRaw / 65.5;
  giroY = gyRaw / 65.5;
  giroZ = gzRaw / 65.5;

  datosMPUValidos = true;
}

// =====================================================
// LECTURA MAX30102
// =====================================================
void actualizarMAX30102() {
  if (!statusMAX30102) {
    return;
  }

  sensorMAX.check();

  while (sensorMAX.available()) {
    valorRojo = sensorMAX.getRed();
    valorIR = sensorMAX.getIR();

    if (valorIR > 50000) {
      if (checkForBeat(valorIR)) {
        unsigned long tiempoActual = millis();

        if (tiempoUltimoLatido > 0) {
          unsigned long diferencia =
            tiempoActual - tiempoUltimoLatido;

          bpmInstantaneo =
            60.0 / (diferencia / 1000.0);

          if (bpmInstantaneo >= 20 &&
              bpmInstantaneo <= 255) {

            valoresBPM[posicionBPM] =
              (byte)bpmInstantaneo;

            posicionBPM++;

            if (posicionBPM >= CANTIDAD_BPM) {
              posicionBPM = 0;
            }

            if (cantidadBPMValidos < CANTIDAD_BPM) {
              cantidadBPMValidos++;
            }

            int sumaBPM = 0;

            for (byte i = 0;
                 i < cantidadBPMValidos;
                 i++) {

              sumaBPM += valoresBPM[i];
            }

            bpmPromedio =
              sumaBPM / cantidadBPMValidos;
          }
        }

        tiempoUltimoLatido = tiempoActual;
      }
    } else {
      bpmInstantaneo = 0;
      bpmPromedio = 0;
      tiempoUltimoLatido = 0;
      posicionBPM = 0;
      cantidadBPMValidos = 0;
    }

    sensorMAX.nextSample();
  }
}

// =====================================================
// DS18B20 SIN BLOQUEAR EL PROGRAMA
// =====================================================
void solicitarTemperaturaDS() {
  if (!statusDS18B20) {
    return;
  }

  sensorDS18B20.requestTemperatures();

  tiempoSolicitudDS = millis();
  conversionDSActiva = true;
}

void actualizarDS18B20(unsigned long tiempoActual) {
  if (!statusDS18B20 || !conversionDSActiva) {
    return;
  }

  if (tiempoActual - tiempoSolicitudDS
      >= CONVERSION_DS18B20_MS) {

    float lectura =
      sensorDS18B20.getTempCByIndex(0);

    if (lectura != DEVICE_DISCONNECTED_C) {
      temperaturaCabina = lectura;
      temperaturaValida = true;
    } else {
      temperaturaValida = false;
    }

    solicitarTemperaturaDS();
  }
}

// =====================================================
// LECTURA RFID-RC522
// =====================================================
void verificarRFID() {
  if (!statusRFID) {
    return;
  }

  if (!sensorRFID.PICC_IsNewCardPresent()) {
    return;
  }

  if (!sensorRFID.PICC_ReadCardSerial()) {
    return;
  }

  Serial.println();
  Serial.print(">>> TARJETA RFID DETECTADA - UID:");

  for (byte i = 0; i < sensorRFID.uid.size; i++) {
    if (sensorRFID.uid.uidByte[i] < 0x10) {
      Serial.print(" 0");
    } else {
      Serial.print(" ");
    }

    Serial.print(
      sensorRFID.uid.uidByte[i],
      HEX
    );
  }

  Serial.println(" <<<");
  Serial.println();

  sensorRFID.PICC_HaltA();
  sensorRFID.PCD_StopCrypto1();
}

// =====================================================
// REPORTE GENERAL
// =====================================================
void mostrarTelemetria() {
  Serial.println();
  Serial.println(
    "--- TELEMETRIA DEL NODO BIOMEDICO ---"
  );

  // DS18B20
  if (!statusDS18B20) {
    Serial.println(
      "Temperatura : SENSOR NO DETECTADO"
    );
  } else if (!temperaturaValida) {
    Serial.println(
      "Temperatura : Esperando lectura"
    );
  } else {
    Serial.printf(
      "Temperatura : %.2f grados C\n",
      temperaturaCabina
    );
  }

  // MAX30102
  if (!statusMAX30102) {
    Serial.println(
      "MAX30102    : SENSOR NO DETECTADO"
    );
  } else if (valorIR <= 50000) {
    Serial.printf(
      "MAX30102    : Sin contacto | IR=%lu\n",
      (unsigned long)valorIR
    );
  } else {
    Serial.printf(
      "PPG         : RED=%lu | IR=%lu\n",
      (unsigned long)valorRojo,
      (unsigned long)valorIR
    );

    if (bpmPromedio > 0) {
      Serial.printf(
        "Pulso       : %.1f BPM | Promedio=%d BPM\n",
        bpmInstantaneo,
        bpmPromedio
      );
    } else {
      Serial.println(
        "Pulso       : Calculando..."
      );
    }
  }

  // MPU6500
  if (!statusMPU6500) {
    Serial.println(
      "MPU6500     : SENSOR NO DETECTADO"
    );
  } else if (!datosMPUValidos) {
    Serial.println(
      "MPU6500     : Esperando lectura"
    );
  } else {
    Serial.printf(
      "Aceleracion : X=%.2f | Y=%.2f | Z=%.2f m/s2\n",
      aceleracionX,
      aceleracionY,
      aceleracionZ
    );

    Serial.printf(
      "Giroscopio  : X=%.2f | Y=%.2f | Z=%.2f grados/s\n",
      giroX,
      giroY,
      giroZ
    );
  }

  // RC522
  if (statusRFID) {
    Serial.println(
      "RFID        : Listo, acerque una tarjeta"
    );
  } else {
    Serial.println(
      "RFID        : SENSOR NO DETECTADO"
    );
  }

  Serial.println(
    "-------------------------------------"
  );
}