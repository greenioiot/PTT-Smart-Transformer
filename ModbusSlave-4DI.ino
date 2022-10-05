#include <ModbusSlave.h>
#include <HardwareSerial.h>

#define in1PIN  32
#define in2PIN  33
#define in3PIN  25
#define in4PIN  26

/* slave id = 1, control-pin = 7, baud = 9600
*/
#define SLAVE_ID 3
#define CTRL_PIN 27
#define BAUDRATE 9600

#define ledStat 0

int pinIn[4] = {in1PIN, in2PIN, in3PIN, in4PIN};

HardwareSerial MySerial(1);

/**
    Modbus object declaration.
*/
Modbus slave(MySerial, SLAVE_ID, CTRL_PIN);

void setup()
{
  for (int a = 0; a < 4 ; a++)
  {
    pinMode(pinIn[a], INPUT);
  }

  pinMode(ledStat, OUTPUT);   digitalWrite(ledStat, HIGH);

  /* register handler functions.
     into the modbus slave callback vector.
  */
  slave.cbVector[CB_READ_HOLDING_REGISTERS] = readDigital;
//  slave.cbVector[CB_READ_COILS] = readDigital;
  Serial.begin( BAUDRATE );
  MySerial.begin(9600, SERIAL_8N1, 16, 17);
  slave.begin( BAUDRATE );

}

void loop()
{
  /* listen for modbus commands con serial port

     on a request, handle the request.
     if the request has a user handler function registered in cbVector
     call the user handler function.
  */
  slave.poll();
}


/**
   Handel Read Input Status (FC=01/02)
   write back the values from digital pins (input status).

   handler functions must return void and take:
        uint8_t  fc - function code.
        uint16_t address - first register/coil address.
        uint16_t length/status - length of data / coil status.
*/
uint8_t readDigital(uint8_t fc, uint16_t address, uint16_t length) {
  // read digital input
  for (int i = 0; i < length; i++) {
    // write one boolean (1 bit) to the response buffer.
    slave.writeRegisterToBuffer(i, !digitalRead(pinIn[address + i]));
  }

  return STATUS_OK;
}
