# Proyecto 2: Estación Inalámbrica de Bioseguridad y Control de Acceso para Incubadoras Neonatales

**Integrantes:** Benjamin Catalán, Marco Díaz
**Curso:** CBM413 - Laboratorio de Electromedicina III

## Pinout
- DS18B20: OneWire GPIO4 (pull-up 4.7k a 3.3V)
- MAX30102: I2C SDA=21, SCL=22 (0x57)
- MPU6500/6050: I2C SDA=21, SCL=22 (0x68/0x70)
- RC522: SPI SCK=18, MISO=19, MOSI=23, CS=5, RST=27

## Estado del avance
- [x] Lectura simultánea de los 4 sensores
- [x] RFID lee UID
- [x] Adquisición sin delay() en loop