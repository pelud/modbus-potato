#ifndef __ModbusArduinoHardwareSerial_h__
#define __ModbusArduinoHardwareSerial_h__
#include "ModbusInterface.h"
#ifdef ARDUINO
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
        /// port_number corresponds to the given Serial object.
        /// i.e. port 0 is Serial, port 1 is Serial1, etc.
        /// </remarks>
        CModbusArduinoHardwareSerial(uint8_t port_number);

        virtual int read(uint8_t* buffer, size_t buffer_size);
        virtual int write(uint8_t* buffer, size_t len);
        virtual void txEnable(bool state);
        virtual bool writeComplete();
        virtual void communicationStatus(bool rx, bool tx) {}
    private:
        int availableForWrite();
        HardwareSerial* m_serial;
        uint8_t m_port_number;
    };
}
#endif
#endif