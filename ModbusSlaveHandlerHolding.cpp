#include "ModbusSlaveHandlerHolding.h"
namespace ModbusPotato
{
    CModbusSlaveHandlerHolding::CModbusSlaveHandlerHolding(uint16_t* array, size_t len)
        :  m_array(array)
        ,  m_len(len)
    {
    }

    modbus_exception_code::modbus_exception_code CModbusSlaveHandlerHolding::read_holding_registers(uint16_t address, uint16_t count, uint16_t* result)
    {
        // check to make sure the address and count are valid
        //
        // Note: The address starts at 0 for the first holding register (40001)
        //
        if (count > m_len || address >= m_len || (size_t)(address + count) > m_len)
            return modbus_exception_code::illegal_data_address;

        // copy the values
        for (; count; address++, count--)
            *result++= m_array[address];

        return modbus_exception_code::ok;
    }

    modbus_exception_code::modbus_exception_code CModbusSlaveHandlerHolding::write_multiple_registers(uint16_t address, uint16_t count, const uint16_t* values)
    {
        // check to make sure the address and count are valid
        //
        // Note: The address starts at 0 for the first holding register (40001)
        //
        if (count > m_len || address >= m_len || (size_t)(address + count) > m_len)
            return modbus_exception_code::illegal_data_address;

        // copy the values
        for (; count; ++address, --count)
            m_array[address] = *values++;

        return modbus_exception_code::ok;
    }
}
