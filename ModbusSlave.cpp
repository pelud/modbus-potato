#include "ModbusSlave.h"
namespace ModbusPotato
{
    CModbusSlave::CModbusSlave(ISlaveHandler* handler)
        :   m_handler(handler)
    {
    }

    void CModbusSlave::frame_ready(IFramer* framer)
    {
        // check if the function code is missing
        if (!framer->buffer_len())
        {
            // if so, acknowledge and exit as we can't send back an exception without it
            framer->finished();
            return;
        }

        // lock the buffer
        if (!framer->begin_send())
            return; // collision

        // handle the function code
        //
        // See http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
        //
        uint8_t result = modbus_exception_code::illegal_function;
        if (m_handler)
        {
            switch (framer->buffer()[0])
            {
            case read_coil_status:
                result = read_bit_input_rsp(framer, false);
                break;
            case read_discrete_input_status:
                result = read_bit_input_rsp(framer, true);
                break;
            case read_holding_registers:
                result = read_registers_rsp(framer, true);
                break;
            case read_input_registers:
                result = read_registers_rsp(framer, false);
                break;
            case write_single_coil:
                result = write_single_coil_rsp(framer);
                break;
            case write_single_register:
                result = write_single_register_rsp(framer);
                break;
            case write_multiple_coils:
                result = write_multiple_coils_rsp(framer);
                break;
            case write_multiple_registers:
                result = write_multiple_registers_rsp(framer);
                break;
            }
        }

        // exit if this is a broadcast packet (no response needed)
        if (framer->station_address() && !framer->frame_address())
        {
            framer->finished();
            return;
        }

        // check if an exception code was returned
        if (result != 0)
        {
            // generate the exception packet
            uint8_t* buffer = framer->buffer();
            buffer[0] |= 0x80;
            buffer[1] = result;
            framer->set_buffer_len(2);
        }

        // send the result back
        framer->send();
    }

    uint8_t CModbusSlave::read_bit_input_rsp(IFramer* framer, bool discrete)
    {
        if (framer->buffer_len() != 5)
            return modbus_exception_code::illegal_function;

        // determine the address and count
        uint8_t* buffer = framer->buffer();
        uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
        uint16_t count = ((uint16_t)buffer[3] << 8) | buffer[4];
        
        // determine the resulting buffer length
        //
        // buffer[0] = fc
        // buffer[1] = byte count
        // buffer[2+] = data
        //
        size_t bytes = (count + 7) / 8;
        size_t buffer_len = bytes + 2;

        // check to make sure the count is valid
        if (count < 1 || buffer_len > framer->buffer_max())
            return modbus_exception_code::illegal_data_value; // count not valid

        // execute the handler
        uint8_t result = modbus_exception_code::illegal_function;
        if (discrete)
            result = m_handler->read_discrete_inputs(address, count, buffer + 2);
        else
            result = m_handler->read_coils(address, count, buffer + 2);

        // check if something went wrong
        if (result != modbus_exception_code::ok)
            return result; // error

        // update the byte count and packet length
        buffer[1] = bytes;
        framer->set_buffer_len(buffer_len);

        return modbus_exception_code::ok;
    }

    uint8_t CModbusSlave::read_registers_rsp(IFramer* framer, bool holding)
    {
        // make sure the function code and handler are valid
        if (framer->buffer_len() != 5)
            return modbus_exception_code::illegal_function;

        // determine the address and count
        uint8_t* buffer = framer->buffer();
        uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
        uint16_t count = ((uint16_t)buffer[3] << 8) | buffer[4];

        // determine the resulting buffer length
        //
        // buffer[0] = fc
        // buffer[1] = byte count
        // buffer[2+] = data
        //
        size_t buffer_len = count * 2 + 2;

        // check to make sure the count is valid
        if (!count || buffer_len > framer->buffer_max())
            return modbus_exception_code::illegal_data_value; // count not valid

        // get the pointer into the buffer for the resulting data
        uint16_t* regs = (uint16_t*)(buffer + 2);

        // execute the handler
        uint8_t result = modbus_exception_code::illegal_function;
        if (holding)
            result = m_handler->read_holding_registers(address, count, regs);
        else
            result = m_handler->read_input_registers(address, count, regs);

        // check if something went wrong
        if (result != modbus_exception_code::ok)
            return result; // error

        // set the resulting byte count and buffer length
        buffer[1] = count * 2;
        framer->set_buffer_len(buffer_len);

        // fixup the byte order of the resulting registers
        for (uint16_t i = 0; i < count; ++i)
            regs[i] = htons(regs[i]);

        return modbus_exception_code::ok;
    }

    uint8_t CModbusSlave::write_single_coil_rsp(IFramer* framer)
    {
        if (framer->buffer_len() != 5)
            return modbus_exception_code::illegal_function;

        // determine the address and value
        uint8_t* buffer = framer->buffer();
        uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
        uint16_t value = ((uint16_t)buffer[3] << 8) | buffer[4];

        // make sure the value is valid
        if (value != 0 && value != 0xff00)
            return modbus_exception_code::illegal_data_value;

        // execute the handler
        return m_handler->write_single_coil(address, value != 0);
    }

    uint8_t CModbusSlave::write_single_register_rsp(IFramer* framer)
    {
        if (framer->buffer_len() != 5)
            return modbus_exception_code::illegal_function;

        // determine the address and value
        uint8_t* buffer = framer->buffer();
        uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
        uint16_t value = ((uint16_t)buffer[3] << 8) | buffer[4];

        // execute the handler
        return m_handler->write_single_register(address, value);
    }

    uint8_t CModbusSlave::write_multiple_coils_rsp(IFramer* framer)
    {
        if (framer->buffer_len() < 6)
            return modbus_exception_code::illegal_function;

        // determine the address and count
        uint8_t* buffer = framer->buffer();
        uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
        uint16_t count = ((uint16_t)buffer[3] << 8) | buffer[4];
        uint8_t check = buffer[5];

        // make sure the counts are valid
        if (count < 0 || (count + 7) / 8 != check || check + 6 != framer->buffer_len())
            return modbus_exception_code::illegal_data_value;

        // execute the handler
        if (uint8_t result = m_handler->write_multiple_coils(address, count, buffer + 6))
            return result; // error

        // set the result buffer
        buffer[1] = (uint8_t)(address >> 8);
        buffer[2] = (uint8_t)address;
        buffer[3] = (uint8_t)(count >> 8);
        buffer[4] = (uint8_t)count;
        framer->set_buffer_len(5);

        return modbus_exception_code::ok;
    }

    uint8_t CModbusSlave::write_multiple_registers_rsp(IFramer* framer)
    {
        // see Figure 22 of http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
        if (framer->buffer_len() < 6)
            return modbus_exception_code::illegal_function;

        // determine the address and count
        uint8_t* buffer = framer->buffer();
        uint16_t address = ((uint16_t)buffer[1] << 8) | buffer[2];
        uint16_t count = ((uint16_t)buffer[3] << 8) | buffer[4];
        uint8_t check = buffer[5];

        // make sure the counts are valid
        if (count < 0 || count * 2 != check || check + 6 != framer->buffer_len())
            return modbus_exception_code::illegal_data_value;

        // get the pointer into the buffer for the resulting data
        uint16_t* regs = (uint16_t*)(buffer + 6);

        // fixup the byte order of the registers
        for (uint16_t i = 0; i < count; ++i)
            regs[i] = htons(regs[i]);

        // execute the handler
        if (uint8_t result = m_handler->write_multiple_registers(address, count, regs))
            return result; // error

        // set the result buffer
        buffer[1] = (uint8_t)(address >> 8);
        buffer[2] = (uint8_t)address;
        buffer[3] = (uint8_t)(count >> 8);
        buffer[4] = (uint8_t)count;
        framer->set_buffer_len(5);

        return modbus_exception_code::ok;
    }
}
