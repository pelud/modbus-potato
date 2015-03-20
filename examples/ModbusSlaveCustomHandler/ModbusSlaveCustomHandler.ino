// This example demonstrates a custom handler for a Modbus slave device.
//
#include <ModbusRTU.h>
#include <ModbusSlave.h>
#include <ModbusSlaveHandlerBase.h>
#include <ModbusArduinoHardwareSerial.h>
#include <ModbusArduinoTimeProvider.h>
using namespace ModbusPotato;

#define LED_PIN (13)
#define SLAVE_ADDRESS (1)
#define BAUD_RATE (19200)

static uint16_t m_brightness = 0x8000; // initial value of 50% brightness
static bool m_visible = true; // true if the LED is turned on
static uint16_t m_phaseaccum = 0; // phase accumulator for PWM of the LED

// this class implements the callbacks to read and write the actual data
class CSlaveHandler : public CModbusSlaveHandlerBase
{
  public:

    // read multiple coils
    virtual modbus_exception_code::modbus_exception_code read_coils(uint16_t address, uint16_t count, uint8_t* result)
    {
      // address 0 starts at the first coil
      for (uint16_t b = 0; b < count; address++, b++)
      {
        bool value = false;
        switch (address)
        {
          case 0: // 1
            value = m_visible;
            break;
          default:
            return modbus_exception_code::illegal_data_address;
        }

        if (value)
          result[b >> 3] |= 1 << (b & 7);
        else
          result[b >> 3] &= ~(1 << (b & 7));
      }
      return modbus_exception_code::ok;
    }

    // write multiple coils
    virtual modbus_exception_code::modbus_exception_code write_multiple_coils(uint16_t address, uint16_t count, const uint8_t* values)
    {
      // address 0 starts at the first coil
      for (uint16_t b = 0; b < count; address++, b++)
      {
        bool value = (values[b >> 3] & (1 << (b & 7))) != 0;
        switch (address)
        {
          case 0: // 1
            m_visible = value;
            break;
          default:
            return modbus_exception_code::illegal_data_address;
        }
      }
      return modbus_exception_code::ok;
    }

    // read a holding register
    virtual modbus_exception_code::modbus_exception_code read_holding_registers(uint16_t address, uint16_t count, uint16_t* result)
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
            return modbus_exception_code::illegal_data_address;
        }
      }
      return modbus_exception_code::ok;
    }

    // write a holding register
    virtual modbus_exception_code::modbus_exception_code write_multiple_registers(uint16_t address, uint16_t count, const uint16_t* values)
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
            return modbus_exception_code::illegal_data_address;
        }
      }
      return modbus_exception_code::ok;
    }
};

// chain together the class implementations
// for Serial, leave as driver(0);
// for Serial1, change to driver(1);
// for Serial2, change to driver(2); etc
static CModbusArduinoHardwareSerial driver(0);
static CModbusArduinoTimeProvider time_provider;
static uint8_t m_frame_buffer[MODBUS_DATA_BUFFER_SIZE];
static CModbusRTU rtu(&driver, &time_provider, m_frame_buffer, MODBUS_DATA_BUFFER_SIZE);
static CSlaveHandler slave_handler;
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
  bool carry = last > (m_phaseaccum += m_brightness);
  digitalWrite(LED_PIN, m_visible && carry ? HIGH : LOW);
}

