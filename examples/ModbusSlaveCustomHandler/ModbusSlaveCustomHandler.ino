#include <ModbusRTU.h>
#include <ModbusSlave.h>
#include <ModbusSlaveHandlerBase.h>
#include <ModbusArduinoHardwareSerial.h>
#include <ModbusArduinoTimeProvider.h>

#define LED_PIN (13)
#define SLAVE_ADDRESS (1)
#define BAUD_RATE (19200)

static uint16_t m_brightness = 0x8000; // initial value of 50% brightness
static uint16_t m_phaseaccum = 0; // phase accumulator for PWM on led

// this class implements the callbacks to read and write the actual data
class CSlaveHandler : public ModbusPotato::CModbusSlaveHandlerBase
{
  public:

    // read a holding register
    virtual ModbusPotato::modbus_exception_code::modbus_exception_code read_holding_registers(uint16_t address, uint16_t count, uint16_t* result)
    {
      for (; count; address++, count--)
      {
        // Note: The address starts at 0 for the first holding register (40001)
        switch (address)
        {
          case 0: // 40001
            *result++ = m_brightness;
            break;
          default:
            return ModbusPotato::modbus_exception_code::illegal_data_address;
        }
      }
      return ModbusPotato::modbus_exception_code::ok;
    }

    // write a holding register
    virtual ModbusPotato::modbus_exception_code::modbus_exception_code write_multiple_registers(uint16_t address, uint16_t count, const uint16_t* values)
    {
      for (; count; ++address, --count)
      {
        // Note: The address starts at 0 for the first holding register (40001)
        switch (address)
        {
          case 0: // 40001
            m_brightness = *values++;
            break;
          default:
            return ModbusPotato::modbus_exception_code::illegal_data_address;
        }
      }
      return ModbusPotato::modbus_exception_code::ok;
    }
};

// chain together the class implementations
// for Serial1, change to driver(&Serial1, &UCSR1A, &UCSR1B),
// for Serial2, change to driver(&Serial2, &UCSR2A, &UCSR2B), etc
static ModbusPotato::CModbusArduinoHardwareSerial driver(&Serial, &UCSR0A, &UCSR0B);
static ModbusPotato::CModbusArduinoTimeProvider time_provider;
static ModbusPotato::CModbusRTU rtu(&driver, &time_provider);
static CSlaveHandler slave_handler;
static ModbusPotato::CModbusSlave slave(&rtu, &slave_handler);

void setup() {

  // initialize the modbus library
  Serial.begin(BAUD_RATE, SERIAL_8E1);
  rtu.setup(BAUD_RATE);
  rtu.set_station_address(SLAVE_ADDRESS);
  rtu.set_handler(&slave);

  // initialize digital pin 13 as an output.
  pinMode(LED_PIN, OUTPUT);

  // flash the LED a few times to indicate that we have booted
  for (int i = 0; i < 4; ++i)
  {
    digitalWrite(LED_PIN, LOW);
    delay(250);
    digitalWrite(LED_PIN, HIGH);
    delay(250);
  }
  digitalWrite(LED_PIN, LOW);
}

void loop() {

  // poll the modbus library
  rtu.poll();

  // perform the LED PWM
  uint16_t last = m_phaseaccum;
  digitalWrite(LED_PIN, last > (m_phaseaccum += m_brightness) ? HIGH : LOW);
}

