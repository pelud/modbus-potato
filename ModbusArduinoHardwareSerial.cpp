#include "ModbusArduinoHardwareSerial.h"
namespace ModbusPotato
{
    CModbusArduinoHardwareSerial::CModbusArduinoHardwareSerial(HardwareSerial* serial, volatile uint8_t *ucsra, volatile uint8_t *ucsrb)
        :  m_serial(serial)
        ,  m_ucsra(ucsra)
        ,  m_ucsrb(ucsrb)
    {
    }

    int CModbusArduinoHardwareSerial::read(uint8_t* buffer, size_t buffer_size)
    {
        // check how many characters are available
        int a = m_serial->available();
        if (a <= 0)
            return a; // nothing to do

        // make sure we don't over-run the end of the buffer
        if (buffer_size != (size_t)-1 && a > buffer_size)
            a = buffer_size;

        // if there is no buffer provided, then dump the input and return the number of characters dumped
        if (!buffer)
        {
            for (int i = 0; i < a; ++i)
                m_serial->read();
            return a;
        }

        // read the data
        return m_serial->readBytes(buffer, a);
    }

    int CModbusArduinoHardwareSerial::write(uint8_t* buffer, size_t len)
    {
        // check how many characters can be written without blocking
        int a = m_serial->availableForWrite();
        if (a <= 0)
            return a; // nothing to do

        // limit the amount to write based on the available space
        if (len > a)
            len = a;

        // write the data
        return m_serial->write(buffer, len);
    }

    void CModbusArduinoHardwareSerial::txEnable(bool state)
    {
        // TODO: over-ride this to enable or disable the RS-485 transceiver, if needed
    }

    bool CModbusArduinoHardwareSerial::writeComplete()
    {
        // if empty, check the if the interrupt UDRE interrupt handler is enabled UART TX Complete bit
        //
        // Note: this assumes that the HardwareSerial driver works a specific
        // way.  If the driver changes, you may need to look the flush() method
        // to see the proper mechanism to determine if the transmit has
        // completed.
        //
        // The HardwareSerial driver turns on the UDRE interrupt enable bit if
        // there was something placed in the FIFO, and turns it off in the
        // interrupt handler when the FIFO is empty.  Therefore we only need to
        // check this bit instead of the FIFO to see if all data was written to
        // the UDR register.
        //
        return bit_is_clear(m_ucsrb, UDRIE0) && bit_is_set(m_ucsra, TXC0);
    }
}
