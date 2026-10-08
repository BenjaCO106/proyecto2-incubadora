#include <Wire.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <MFRC522.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// =====================================================
// PINES UTILIZADOS
// =====================================================

#define PIN_I2C_SDA       21
#define PIN_I2C_SCL       22

#define PIN_ONEWIRE        4

#define PIN_RC522_CS       5
#define PIN_RC522_RST     27
#define PIN_SPI_SCK       18
#define PIN_SPI_MISO      19
#define PIN_SPI_MOSI      23

#define PIN_LED_VERDE     25
#define PIN_LED_ROJO      26

// =====================================================
// REGISTROS DEL MPU6500
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
// CONFIGURACION GENERAL
// =====================================================

const uint32_t UMBRAL_CONTACTO_IR = 50000;
const byte CANTIDAD_BPM = 4;

// Este es el llavero que quedara autorizado.
const byte UID_AUTORIZADO[] = {0x1A, 0x86, 0x2A, 0x07};
const byte LARGO_UID_AUTORIZADO = sizeof(UID_AUTORIZADO);

// Los LED permanecen encendidos este tiempo despues de leer una credencial.
const TickType_t TIEMPO_LED = pdMS_TO_TICKS(1500);

// =====================================================
// OBJETOS DE LOS SENSORES
// =====================================================

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensorDS18B20(&oneWire);
MAX30105 sensorMAX;
MFRC522 sensorRFID(PIN_RC522_CS, PIN_RC522_RST);

// =====================================================
// ESTADO DE INICIALIZACION
// =====================================================

bool statusDS18B20 = false;
bool statusMAX30102 = false;
bool statusMPU6500 = false;
bool statusRFID = false;

// =====================================================
// VARIABLES COMPARTIDAS ENTRE TAREAS
// =====================================================

float temperaturaCabina = 0.0;
bool temperaturaValida = false;

uint32_t valorRojo = 0;
uint32_t valorIR = 0;
float bpmInstantaneo = 0.0;
float bpmPromedio = 0.0;

float aceleracionX = 0.0;
float aceleracionY = 0.0;
float aceleracionZ = 0.0;
float giroX = 0.0;
float giroY = 0.0;
float giroZ = 0.0;
bool datosMPUValidos = false;

// Mutex para que una tarea no lea los datos mientras otra los modifica.
SemaphoreHandle_t mutexDatos;

// Mutex para evitar que dos tareas escriban al mismo tiempo en el monitor serial.
SemaphoreHandle_t mutexSerial;

// =====================================================
// VARIABLES INTERNAS PARA EL CALCULO DE BPM
// =====================================================

float valoresBPM[CANTIDAD_BPM] = {0};
byte posicionBPM = 0;
byte cantidadBPMValidos = 0;
unsigned long tiempoUltimoLatido = 0;

// =====================================================
// DECLARACION DE FUNCIONES
// =====================================================

bool escribirRegistroMPU(byte registro, byte valor);
byte leerRegistroMPU(byte registro);
bool leerBloqueMPU(byte registroInicial, byte *datos, byte cantidad);
bool inicializarMPU6500();

void actualizarMAX30102();
void actualizarMPU6500();
void procesarTarjetaRFID();
bool esTarjetaAutorizada(const MFRC522::Uid &uid);
void indicarAcceso(bool autorizado);
void apagarIndicadores();
void mostrarTelemetria();

void tareaSensoresI2C(void *parametro);
void tareaTemperatura(void *parametro);
void tareaRFID(void *parametro);
void tareaTelemetria(void *parametro);

// =====================================================
// CONFIGURACION INICIAL
// =====================================================

void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.println("==========================================");
  Serial.println(" ESTACION NEONATAL - ESP32 CON FREERTOS");
  Serial.println("==========================================");

  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_ROJO, OUTPUT);
  apagarIndicadores();

  mutexDatos = xSemaphoreCreateMutex();
  mutexSerial = xSemaphoreCreateMutex();

  if (mutexDatos == NULL || mutexSerial == NULL) {
    Serial.println("[ERROR] No se pudieron crear los mutex");
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  // MAX30102 y MPU6500 comparten estas dos lineas I2C.
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  // ---------------------------------------------------
  // DS18B20
  // ---------------------------------------------------

  sensorDS18B20.begin();

  if (sensorDS18B20.getDeviceCount() > 0) {
    statusDS18B20 = true;
    sensorDS18B20.setResolution(12);

    // La conversion se inicia y la tarea espera sin detener las otras tareas.
    sensorDS18B20.setWaitForConversion(false);
    Serial.println("[OK] DS18B20 detectado");
  } else {
    Serial.println("[ERROR] DS18B20 no detectado");
  }

  // ---------------------------------------------------
  // MAX30102
  // ---------------------------------------------------

  if (sensorMAX.begin(Wire, I2C_SPEED_FAST)) {
    statusMAX30102 = true;

    // Potencia LED, promedio, modo rojo+IR, 100 muestras/s,
    // ancho de pulso de 411 us y rango ADC de 4096 nA.
    sensorMAX.setup(0x1F, 4, 2, 400, 411, 4096);
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

  Serial.println("==========================================");
  Serial.println(" CREANDO TAREAS FREERTOS");
  Serial.println("==========================================");

  // El MAX30102 y el MPU6500 se atienden en una misma tarea.
  // Asi nunca intentan ocupar el bus I2C simultaneamente.
  xTaskCreatePinnedToCore(
    tareaSensoresI2C,
    "Sensores_I2C",
    4096,
    NULL,
    3,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    tareaTemperatura,
    "Temperatura",
    3072,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    tareaRFID,
    "Control_RFID",
    3072,
    NULL,
    2,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    tareaTelemetria,
    "Telemetria",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  Serial.println("[OK] Sistema iniciado");
}

// Arduino ejecuta loop() dentro de una tarea propia. Las funciones del
// proyecto se ejecutan en las cuatro tareas creadas en setup().
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// =====================================================
// TAREA 1: MAX30102 Y MPU6500 - NUCLEO 1
// =====================================================

void tareaSensoresI2C(void *parametro) {
  TickType_t ultimoCiclo = xTaskGetTickCount();
  TickType_t ultimaLecturaMPU = ultimoCiclo;

  while (true) {
    if (statusMAX30102) {
      actualizarMAX30102();
    }

    TickType_t ahora = xTaskGetTickCount();

    // El MPU6500 se actualiza cada 100 ms, equivalente a 10 lecturas/s.
    if (statusMPU6500 &&
        (ahora - ultimaLecturaMPU >= pdMS_TO_TICKS(100))) {
      ultimaLecturaMPU = ahora;
      actualizarMPU6500();
    }

    // La tarea se duerme hasta completar un periodo de 5 ms.
    // Esto permite atender el FIFO del MAX30102 sin detener el programa.
    vTaskDelayUntil(&ultimoCiclo, pdMS_TO_TICKS(5));
  }
}

// =====================================================
// TAREA 2: DS18B20 - NUCLEO 1
// =====================================================

void tareaTemperatura(void *parametro) {
  while (true) {
    if (!statusDS18B20) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Se solicita la medicion y solo esta tarea espera la conversion.
    // Las tareas de pulso, movimiento y acceso siguen funcionando.
    sensorDS18B20.requestTemperatures();
    vTaskDelay(pdMS_TO_TICKS(750));

    float lectura = sensorDS18B20.getTempCByIndex(0);
    bool lecturaValida = (lectura != DEVICE_DISCONNECTED_C);

    xSemaphoreTake(mutexDatos, portMAX_DELAY);
    if (lecturaValida) {
      temperaturaCabina = lectura;
    }
    temperaturaValida = lecturaValida;
    xSemaphoreGive(mutexDatos);
  }
}

// =====================================================
// TAREA 3: CONTROL DE ACCESO RFID - NUCLEO 0
// =====================================================

void tareaRFID(void *parametro) {
  // El bus SPI y el lector se inicializan en esta misma tarea.
  // Asi todas las operaciones del RC522 se ejecutan en el nucleo 0.
  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_RC522_CS);
  sensorRFID.PCD_Init();
  vTaskDelay(pdMS_TO_TICKS(50));

  // Se enciende la antena y se usa la ganancia maxima del receptor.
  sensorRFID.PCD_AntennaOn();
  sensorRFID.PCD_SetAntennaGain(MFRC522::RxGain_max);

  byte versionRC522 = sensorRFID.PCD_ReadRegister(sensorRFID.VersionReg);

  if (versionRC522 != 0x00 && versionRC522 != 0xFF) {
    statusRFID = true;

    xSemaphoreTake(mutexSerial, portMAX_DELAY);
    Serial.print("[OK] RC522 iniciado en tarea - Version: 0x");
    Serial.println(versionRC522, HEX);
    xSemaphoreGive(mutexSerial);
  } else {
    statusRFID = false;

    xSemaphoreTake(mutexSerial, portMAX_DELAY);
    Serial.println("[ERROR] RC522 no responde dentro de la tarea");
    xSemaphoreGive(mutexSerial);
  }

  TickType_t finIndicacion = 0;
  bool indicadorActivo = false;

  while (true) {
    if (statusRFID &&
        sensorRFID.PICC_IsNewCardPresent() &&
        sensorRFID.PICC_ReadCardSerial()) {

      bool autorizado = esTarjetaAutorizada(sensorRFID.uid);
      indicarAcceso(autorizado);

      indicadorActivo = true;
      finIndicacion = xTaskGetTickCount() + TIEMPO_LED;

      xSemaphoreTake(mutexSerial, portMAX_DELAY);
      Serial.println();
      Serial.print("RFID UID:");

      for (byte i = 0; i < sensorRFID.uid.size; i++) {
        Serial.print(sensorRFID.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(sensorRFID.uid.uidByte[i], HEX);
      }

      if (autorizado) {
        Serial.println(" -> ACCESO AUTORIZADO");
      } else {
        Serial.println(" -> ACCESO DENEGADO");
      }
      xSemaphoreGive(mutexSerial);

      sensorRFID.PICC_HaltA();
      sensorRFID.PCD_StopCrypto1();
    }

    // Apaga los LED al cumplirse el tiempo sin detener esta tarea.
    if (indicadorActivo &&
        (int32_t)(xTaskGetTickCount() - finIndicacion) >= 0) {
      apagarIndicadores();
      indicadorActivo = false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =====================================================
// TAREA 4: REPORTE SERIAL - NUCLEO 0
// =====================================================

void tareaTelemetria(void *parametro) {
  TickType_t ultimoReporte = xTaskGetTickCount();

  while (true) {
    mostrarTelemetria();
    vTaskDelayUntil(&ultimoReporte, pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// FUNCIONES DEL MPU6500
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

  Wire.requestFrom((uint8_t)MPU6500_ADDR, (uint8_t)1, true);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

bool leerBloqueMPU(byte registroInicial, byte *datos, byte cantidad) {
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

  escribirRegistroMPU(REG_PWR_MGMT_1, 0x80);

  // vTaskDelay suspende la tarea actual, no paraliza el procesador.
  vTaskDelay(pdMS_TO_TICKS(100));

  escribirRegistroMPU(REG_PWR_MGMT_1, 0x01);
  escribirRegistroMPU(REG_PWR_MGMT_2, 0x00);
  escribirRegistroMPU(REG_SMPLRT_DIV, 0x09);
  escribirRegistroMPU(REG_CONFIG, 0x04);
  escribirRegistroMPU(REG_GYRO_CONFIG, 0x08);
  escribirRegistroMPU(REG_ACCEL_CONFIG, 0x10);
  escribirRegistroMPU(REG_ACCEL_CONFIG_2, 0x04);

  return true;
}

void actualizarMPU6500() {
  byte datos[14];

  if (!leerBloqueMPU(REG_ACCEL_XOUT_H, datos, 14)) {
    xSemaphoreTake(mutexDatos, portMAX_DELAY);
    datosMPUValidos = false;
    xSemaphoreGive(mutexDatos);
    return;
  }

  int16_t axRaw = (int16_t)((datos[0] << 8) | datos[1]);
  int16_t ayRaw = (int16_t)((datos[2] << 8) | datos[3]);
  int16_t azRaw = (int16_t)((datos[4] << 8) | datos[5]);

  int16_t gxRaw = (int16_t)((datos[8] << 8) | datos[9]);
  int16_t gyRaw = (int16_t)((datos[10] << 8) | datos[11]);
  int16_t gzRaw = (int16_t)((datos[12] << 8) | datos[13]);

  // Para el rango de +-8 g, 4096 cuentas equivalen a 1 g.
  float ax = (axRaw / 4096.0) * 9.80665;
  float ay = (ayRaw / 4096.0) * 9.80665;
  float az = (azRaw / 4096.0) * 9.80665;

  // Para el rango de +-500 grados/s se usan 65,5 cuentas por grado/s.
  float gx = gxRaw / 65.5;
  float gy = gyRaw / 65.5;
  float gz = gzRaw / 65.5;

  xSemaphoreTake(mutexDatos, portMAX_DELAY);
  aceleracionX = ax;
  aceleracionY = ay;
  aceleracionZ = az;
  giroX = gx;
  giroY = gy;
  giroZ = gz;
  datosMPUValidos = true;
  xSemaphoreGive(mutexDatos);
}

// =====================================================
// LECTURA Y CALCULO DE BPM CON EL MAX30102
// =====================================================

void actualizarMAX30102() {
  sensorMAX.check();

  while (sensorMAX.available()) {
    uint32_t rojoLocal = sensorMAX.getRed();
    uint32_t irLocal = sensorMAX.getIR();

    float bpmLocal = bpmInstantaneo;
    float promedioLocal = bpmPromedio;

    if (irLocal > UMBRAL_CONTACTO_IR) {
      if (checkForBeat(irLocal)) {
        unsigned long tiempoActual = millis();

        if (tiempoUltimoLatido > 0) {
          unsigned long intervaloLatidos = tiempoActual - tiempoUltimoLatido;
          float bpmCalculado = 60000.0 / intervaloLatidos;

          if (bpmCalculado >= 30.0 && bpmCalculado <= 240.0) {
            bpmLocal = bpmCalculado;
            valoresBPM[posicionBPM] = bpmCalculado;
            posicionBPM = (posicionBPM + 1) % CANTIDAD_BPM;

            if (cantidadBPMValidos < CANTIDAD_BPM) {
              cantidadBPMValidos++;
            }

            float suma = 0.0;
            for (byte i = 0; i < cantidadBPMValidos; i++) {
              suma += valoresBPM[i];
            }
            promedioLocal = suma / cantidadBPMValidos;
          }
        }

        tiempoUltimoLatido = tiempoActual;
      }
    } else {
      bpmLocal = 0.0;
      promedioLocal = 0.0;
      tiempoUltimoLatido = 0;
      posicionBPM = 0;
      cantidadBPMValidos = 0;

      for (byte i = 0; i < CANTIDAD_BPM; i++) {
        valoresBPM[i] = 0.0;
      }
    }

    xSemaphoreTake(mutexDatos, portMAX_DELAY);
    valorRojo = rojoLocal;
    valorIR = irLocal;
    bpmInstantaneo = bpmLocal;
    bpmPromedio = promedioLocal;
    xSemaphoreGive(mutexDatos);

    sensorMAX.nextSample();
  }
}

// =====================================================
// CONTROL DE ACCESO
// =====================================================

bool esTarjetaAutorizada(const MFRC522::Uid &uid) {
  if (uid.size != LARGO_UID_AUTORIZADO) {
    return false;
  }

  for (byte i = 0; i < LARGO_UID_AUTORIZADO; i++) {
    if (uid.uidByte[i] != UID_AUTORIZADO[i]) {
      return false;
    }
  }

  return true;
}

void indicarAcceso(bool autorizado) {
  digitalWrite(PIN_LED_VERDE, autorizado ? HIGH : LOW);
  digitalWrite(PIN_LED_ROJO, autorizado ? LOW : HIGH);
}

void apagarIndicadores() {
  digitalWrite(PIN_LED_VERDE, LOW);
  digitalWrite(PIN_LED_ROJO, LOW);
}

// =====================================================
// REPORTE DE LOS DATOS
// =====================================================

void mostrarTelemetria() {
  // Primero se copian los datos bajo el mutex y luego se imprimen.
  // De esta manera el mutex queda ocupado durante muy poco tiempo.
  float temp;
  bool tempValida;
  uint32_t rojo;
  uint32_t ir;
  float bpm;
  float promedio;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  bool mpuValido;

  xSemaphoreTake(mutexDatos, portMAX_DELAY);
  temp = temperaturaCabina;
  tempValida = temperaturaValida;
  rojo = valorRojo;
  ir = valorIR;
  bpm = bpmInstantaneo;
  promedio = bpmPromedio;
  ax = aceleracionX;
  ay = aceleracionY;
  az = aceleracionZ;
  gx = giroX;
  gy = giroY;
  gz = giroZ;
  mpuValido = datosMPUValidos;
  xSemaphoreGive(mutexDatos);

  xSemaphoreTake(mutexSerial, portMAX_DELAY);

  Serial.println();
  Serial.println("Datos Actuales");

  if (!statusDS18B20) {
    Serial.println("Temperatura : SENSOR NO DETECTADO");
  } else if (!tempValida) {
    Serial.println("Temperatura : Esperando lectura");
  } else {
    Serial.printf("Temperatura : %.2f grados C\n", temp);
  }

  if (!statusMAX30102) {
    Serial.println("MAX30102    : SENSOR NO DETECTADO");
  } else if (ir <= UMBRAL_CONTACTO_IR) {
    Serial.printf("MAX30102    : Sin contacto | IR=%lu\n", (unsigned long)ir);
  } else {
    Serial.printf(
      "PPG         : RED=%lu | IR=%lu\n",
      (unsigned long)rojo,
      (unsigned long)ir
    );

    if (promedio > 0.0) {
      Serial.printf("Pulso       : %.1f BPM | Promedio=%.1f BPM\n", bpm, promedio);
    } else {
      Serial.println("Pulso       : Calculando...");
    }
  }

  if (!statusMPU6500) {
    Serial.println("MPU6500     : SENSOR NO DETECTADO");
  } else if (!mpuValido) {
    Serial.println("MPU6500     : Esperando lectura");
  } else {
    Serial.printf("Aceleracion : X=%.2f | Y=%.2f | Z=%.2f m/s2\n", ax, ay, az);
    Serial.printf("Giroscopio  : X=%.2f | Y=%.2f | Z=%.2f grados/s\n", gx, gy, gz);
  }

  if (statusRFID) {
    Serial.println("RFID        : Listo |Acerque su identificación");
  } else {
    Serial.println("RFID        : SENSOR NO DETECTADO");
  }

  Serial.println("-------------------------------------");
  xSemaphoreGive(mutexSerial);
}