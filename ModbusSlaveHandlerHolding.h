#ifndef __ModbusSlaveHandlerHolding_h__
#define __ModbusSlaveHandlerHolding_h__
#include "ModbusSlaveHandlerBase.h"
namespace ModbusPotato
{
    /// <summary>
    /// This class is an example of a slave handler for reading and writing holding registers to an array.
    /// </summary>
    class CModbusSlaveHandlerHolding : public CModbusSlaveHandlerBase
    {
    public:
        CModbusSlaveHandlerHolding(uint16_t* array, size_t len);
        virtual modbus_exception_code::modbus_exception_code read_holding_registers(uint16_t address, uint16_t count, uint16_t* result);
        virtual modbus_exception_code::modbus_exception_code write_multiple_registers(uint16_t address, uint16_t count, const uint16_t* values);
    private:
        uint16_t* m_array;
        size_t m_len;
    };
}
#endif
