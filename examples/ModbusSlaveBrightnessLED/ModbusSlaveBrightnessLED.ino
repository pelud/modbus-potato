#include <ModbusRTU.h>
#include <ModbusSlave.h>
#include <ModbusSlaveHandlerBase.h>
#include <ModbusArduinoTimeProvider.h>

#define LED_PIN (13)

class CArduinoSerialDriver : public ModbusPotato::IStream
{
  public:
    virtual int read(uint8_t* buffer, size_t buffer_size)
    {
      // check how many characters are available
      int a = Serial.available();
      if (a <= 0)
        return a; // nothing to do
      
      // make sure we don't over-run the end of the buffer
      if (buffer_size != (size_t)-1 && a > buffer_size)
        a = buffer_size;

      // if there is no buffer provided, then dump the input and return the number of characters dumped
      if (!buffer)
      {
        for (int i = 0; i < a; ++i)
          Serial.read();
        return a;
      }
        
      // read the data
      return Serial.readBytes(buffer, a);
    }
    virtual int write(uint8_t* buffer, size_t len)
    {
      // check how many characters can be written without blocking
      int a = Serial.availableForWrite();
      if (a <= 0)
        return a; // nothing to do

      // limit the amount to write based on the available space
      if (len > a)
        len = a;

      // write the data
      return Serial.write(buffer, len);
    }
    virtual void txEnable(bool state)
    {
      // TODO: enable or disable the RS-485 transceiver, if needed
    }
    virtual bool writeComplete()
    {
      // check if the write buffer is empty and that the UART is done
      // TODO: check the TXC bit
      return Serial.availableForWrite() == SERIAL_TX_BUFFER_SIZE;
    }
};

uint16_t m_brightness = 0x8000; // initial value of 50% brightness
uint16_t m_phaseaccum = 0; // phase accumulator for PWM on led

class CSlaveHandler : public ModbusPotato::CModbusSlaveHandlerBase
{
  public:
    virtual ModbusPotato::modbus_exception_code::modbus_exception_code read_holding_registers(uint16_t address, uint16_t count, uint16_t* result)
    {
      for (; count; address++, count--)
      {
        switch (address)
        {
          case 0:
            *result++ = m_brightness;
            break;
          default:
            return ModbusPotato::modbus_exception_code::illegal_data_address;
        }
      }
      return ModbusPotato::modbus_exception_code::ok;
    }
    virtual ModbusPotato::modbus_exception_code::modbus_exception_code write_multiple_registers(uint16_t address, uint16_t count, const uint16_t* values)
    {
      for (; count; ++address, --count)
      {
        switch (address)
        {
          case 0:
            m_brightness = *values++;
            break;
          default:
            return ModbusPotato::modbus_exception_code::illegal_data_address;
        }
      }
      return ModbusPotato::modbus_exception_code::ok;
    }
};

static CArduinoSerialDriver driver;
static ModbusPotato::CModbusArduinoTimeProvider time_provider;
static ModbusPotato::CModbusRTU rtu(&driver, &time_provider);
static CSlaveHandler slave_handler;
static ModbusPotato::CModbusSlave slave(&rtu, &slave_handler);

void setup() {

  // initialize the modbus library
  Serial.begin(9600, SERIAL_8E1);
  rtu.setup(9600);
  rtu.set_station_address(1);
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
  digitalWrite(LED_PIN, m_brightness && last > (m_phaseaccum += m_brightness) ? HIGH : LOW);
}

