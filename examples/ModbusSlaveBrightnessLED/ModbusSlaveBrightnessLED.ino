// This example demonstrates a Modbus slave using a simple array of holding registers.
//
#include <ModbusRTU.h>
#include <ModbusSlave.h>
#include <ModbusArduinoHardwareSerial.h>
#include <ModbusArduinoTimeProvider.h>
#include <ModbusSlaveHandlerHolding.h>
using namespace ModbusPotato;

#define LED_PIN (13)
#define SLAVE_ADDRESS (1)
#define BAUD_RATE (19200)

#define SLAVE_REGISTER_COUNT (2)
union SlaveRegisterType
{
  struct
  {
    uint16_t brightness; // LED brightness, holding register 1 (40001)
    uint16_t blink_rate; // LED blink rate, half cycle period in milliseconds, holding register 2 (40002)
  } tag;
  uint16_t array[SLAVE_REGISTER_COUNT];
};

static SlaveRegisterType m_registers = { { 0x8000, 500 } }; // initialize with a 50% brightness and 1 second blink rate
static uint16_t m_phaseaccum = 0; // phase accumulator for PWM on led

// chain together the class implementations
// for Serial, leave as driver(0);
// for Serial1, change to driver(1);
// for Serial2, change to driver(2); etc
static CModbusArduinoHardwareSerial driver(0);
static CModbusArduinoTimeProvider time_provider;
static uint8_t m_frame_buffer[MODBUS_DATA_BUFFER_SIZE];
static CModbusRTU rtu(&driver, &time_provider, m_frame_buffer, MODBUS_DATA_BUFFER_SIZE);
static CModbusSlaveHandlerHolding slave_handler(m_registers.array, SLAVE_REGISTER_COUNT);
static CModbusSlave slave(&slave_handler);

void setup() {

  // initialize the modbus library
  Serial.begin(BAUD_RATE, SERIAL_8E1);
  rtu.setup(BAUD_RATE);
  rtu.set_station_address(SLAVE_ADDRESS);
  rtu.set_handler(&slave);

  // initialize digital pin 13 as an output.
  pinMode(LED_PIN, OUTPUT);
}

void loop() {

  // poll the modbus library
  rtu.poll();

  // perform the LED PWM
  uint16_t last = m_phaseaccum;
  bool carry = last > (m_phaseaccum += m_registers.tag.brightness);
  bool nblank = m_registers.tag.blink_rate ? ((millis() / m_registers.tag.blink_rate) & 1 != 0) : true;
  digitalWrite(LED_PIN, nblank && carry ? HIGH : LOW);
}

