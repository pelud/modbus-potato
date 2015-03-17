#include "Slave.h"
namespace ModbusPotato
{
    CSlave::CSlave(IFramer* framer, ISlaveHandler* handler)
        :   m_framer(framer)
        ,   m_handler(handler)
    {
    }

    void CSlave::frame_ready()
    {
        // check if this is a broadcast or missing the function code
        if (!m_framer->frame_address() || !m_framer->buffer_len())
        {
            // if so, acknowledge and exit
            m_framer->finished();
            return;
        }

        // lock the buffer
        if (!m_framer->begin_send())
            return; // collision

        // get the buffer pointer
        uint8_t* buffer = m_framer->buffer();
        
        // handle the function code
        uint8_t result = modbus_exception_code::illegal_function;
        switch (buffer[0])
        {
        case read_holding_registers:
            {
                if (m_framer->buffer_len() == 5 && m_handler)
                {
                    // determine the address and count
                    uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
                    uint16_t count = ((uint16_t)buffer[3] << 8) | buffer[4];

                    // determine the resulting buffer length
                    size_t buffer_len = count * 2 + 2;

                    // check to make sure the count is valid
                    if (buffer_len <= m_framer->buffer_max())
                    {
                        // get the pointer into the buffer for the resulting data
                        uint16_t* regs = (uint16_t*)(buffer + 2);

                        // execute the handler
                        result = m_handler->read_holding_registers(address, count, regs);

                        if (result == modbus_exception_code::ok)
                        {
                            // set the resulting byte count and buffer length
                            buffer[1] = count * 2;
                            m_framer->set_buffer_len(buffer_len);

                            // fixup the byte order of the resulting registers
                            for (uint16_t i = 0; i < count; ++i)
                                regs[i] = htons(regs[i]);
                        }
                    }
                    else
                        result = modbus_exception_code::illegal_data_address; // count not valid
                }
            }
            break;
        case preset_single_register:
            {
                if (m_framer->buffer_len() == 5 && m_handler)
                {
                    // determine the address and value
                    uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
                    uint16_t value = ((uint16_t)buffer[3] << 8) | buffer[4];
                    result = m_handler->preset_single_register(address, value);
                }
            }
            break;
        case write_multiple_registers:
            {
                if (m_framer->buffer_len() >= 6 && m_handler)
                {
                    uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
                    uint16_t count = ((uint16_t)buffer[3] << 8) | buffer[4];
                    uint8_t check = buffer[5];
                    if (count * 2 == check && check + 6 == m_framer->buffer_len())
                    {
                        // get the pointer into the buffer for the resulting data
                        uint16_t* regs = (uint16_t*)(buffer + 6);

                        // fixup the byte order of the registers
                        for (uint16_t i = 0; i < count; ++i)
                            regs[i] = htons(regs[i]);

                        // execute the handler
                        result = m_handler->write_multiple_registers(address, count, regs);

                        if (result == modbus_exception_code::ok)
                        {
                            // set the result buffer
                            buffer[1] = (uint8_t)(address >> 8);
                            buffer[2] = (uint8_t)address;
                            buffer[3] = (uint8_t)(count >> 8);
                            buffer[4] = (uint8_t)count;
                            m_framer->set_buffer_len(5);
                        }
                    }
                }
            }
            break;
        }

        // check if an exception code was returned
        if (result != 0)
        {
            // generate the exception packet
            buffer[0] |= 0x80;
            buffer[1] = result;
            m_framer->set_buffer_len(2);
        }

        // send the result back
        m_framer->send();
    }
}
