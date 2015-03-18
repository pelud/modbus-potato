#ifndef __ModbusArduinoHardwareSerial_h__
#define __ModbusArduinoHardwareSerial_h__
#include "ModbusInterface.h"
namespace ModbusPotato
{
    /// <summary>
    /// This class provides access to the Arduino HardwareSerial driver.
    /// </summary>
    class CModbusArduinoHardwareSerial : public IStream
    {
    public:

        /// <summary>
        /// Construct the interface to the HardwareSerialDriver.
        /// </summary>
        /// <remarks>
        /// The ucsra and ucsrb are the matching serial registers associated
        /// with the given HardwareSerial object.  For example, with the
        /// 'Serial' object, they should be UCSR0A and UCSR0B.  On processors
        /// with Serial1, the registers should be UCSR1A and UCSR1B, etc.
        /// </remarks>
        CModbusArduinoHardwareSerial(HardwareSerial* serial, volatile uint8_t *ucsra, volatile uint8_t *ucsrb);

        virtual int read(uint8_t* buffer, size_t buffer_size);
        virtual int write(uint8_t* buffer, size_t len);
        virtual void txEnable(bool state);
        virtual bool writeComplete();
    private:
        HardwareSerial* m_serial;
        volatile uint8_t * const m_ucsra;
        volatile uint8_t * const m_ucsrb;
    };
}
#endif
