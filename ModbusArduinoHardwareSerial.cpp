#include "ModbusArduinoHardwareSerial.h"
#if defined(ENERGIA) && !defined(SERIAL_BUFFER_SIZE)
#define SERIAL_BUFFER_SIZE 16 // See HardwareSerial.cpp
#endif
#ifdef ARDUINO
namespace ModbusPotato
{
    CModbusArduinoHardwareSerial::CModbusArduinoHardwareSerial(uint8_t port_number)
        :   m_serial()
        ,   m_port_number(port_number)
    {
        switch (port_number)
        {
        case 0:
            m_serial = &Serial;
            break;
#if defined(HAVE_HWSERIAL1) || defined(SERIAL1_AVAILABLE)
        case 1:
            m_serial = &Serial1;
            break;
#endif
#ifdef HAVE_HWSERIAL2
        case 2:
            m_serial = &Serial2;
            break;
#endif
#ifdef HAVE_HWSERIAL3
        case 3:
            m_serial = &Serial3;
            break;
#endif
#ifdef HAVE_HWSERIAL4
        case 4:
            m_serial = &Serial4;
            break;
#endif
#ifdef HAVE_HWSERIAL5
        case 5:
            m_serial = &Serial5;
            break;
#endif
#ifdef HAVE_HWSERIAL6
        case 6:
            m_serial = &Serial6;
            break;
#endif
#ifdef HAVE_HWSERIAL7
        case 7:
            m_serial = &Serial7;
            break;
#endif
#ifdef HAVE_HWSERIAL8
        case 8:
            m_serial = &Serial8;
            break;
#endif
#ifdef HAVE_HWSERIAL9
        case 9:
            m_serial = &Serial9;
            break;
#endif
        default:
            break;
        }
    }

    int CModbusArduinoHardwareSerial::read(uint8_t* buffer, size_t buffer_size)
    {
        if (!m_serial)
            return 0;

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
        return m_serial->readBytes((char*)buffer, a);
    }

    int CModbusArduinoHardwareSerial::write(uint8_t* buffer, size_t len)
    {
        if (!m_serial)
            return 0;

        // check how many characters can be written without blocking
        int a = availableForWrite();
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
#if defined(UBRRH) || defined(UBRR0H) // AVR
        switch (m_port_number)
        {
        case 0:
#ifdef UBRRH
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
            return bit_is_clear(UCSRB, UDRIE0) && bit_is_set(UCSRA, TXC0);
#else
            return bit_is_clear(UCSR0B, UDRIE0) && bit_is_set(UCSR0A, TXC0);
#endif
            break;
#ifdef UBRR1H
        case 1:
            return bit_is_clear(UCSR1B, UDRIE0) && bit_is_set(UCSR1A, TXC0);
            break;
#endif
#ifdef UBRR2H
        case 2:
            return bit_is_clear(UCSR2B, UDRIE0) && bit_is_set(UCSR2A, TXC0);
            break;
#endif
#ifdef UBRR3H
        case 3:
            return bit_is_clear(UCSR3B, UDRIE0) && bit_is_set(UCSR3A, TXC0);
            break;
#endif
        }
#elif defined(__MSP430_HAS_USCI_A0__) || defined(__MSP430_HAS_USCI_A1__) || defined(__MSP430_HAS_EUSCI_A0__) || defined(__MSP430_HAS_EUSCI_A1__) // MSP430
#if defined(__MSP430_HAS_EUSCI_A0__) || defined(__MSP430_HAS_EUSCI_A1__)
#define UCAxIE        UCA0IE_L
#else
#define UCAxIE        UCA0IE
#endif
        // check if the interrupt handler is turned off
        return (*(&(UCAxIE) + m_port_number) & UCTXIE) == 0;
#elif defined(__MSP430_HAS_USCI__) // MSP430
        // check if the interrupt handler is turned off
        return (*(&(UC0IE) + m_port_number) & UCA0TXIE) == 0;
#else
#error CModbusArduinoHardwareSerial::writeComplete() must be tailored to your platform 
#endif
        return false; // invalid port number or not supported by hardware
    }

    int CModbusArduinoHardwareSerial::availableForWrite()
    {
        // return how many characters are available in the write buffer
#ifdef ENERGIA
        // we don't have a direct way to get the number of characters left, so it's all or nothing on ENERGIA
        return writeComplete() ? SERIAL_BUFFER_SIZE : 0;
#else
        return m_serial->availableForWrite();
#endif
    }
}
#endif