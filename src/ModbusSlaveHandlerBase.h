#ifndef __ModbusSlaveHandlerBase_h__
#define __ModbusSlaveHandlerBase_h__
#include "ModbusInterface.h"
namespace ModbusPotato
{
    /// <summary>
    /// This class provides a base class which can be inherited to provide default handlers for slave implementations.
    /// </summary>
    class CModbusSlaveHandlerBase : public ISlaveHandler
    {
    public:
        virtual modbus_exception_code::modbus_exception_code read_coils(uint16_t address, uint16_t count, uint8_t* result) { return modbus_exception_code::illegal_function; }
        virtual modbus_exception_code::modbus_exception_code read_discrete_inputs(uint16_t address, uint16_t count, uint8_t* result) { return modbus_exception_code::illegal_function; }
        virtual modbus_exception_code::modbus_exception_code read_holding_registers(uint16_t address, uint16_t count, uint16_t* result) { return modbus_exception_code::illegal_function; }
        virtual modbus_exception_code::modbus_exception_code read_input_registers(uint16_t address, uint16_t count, uint16_t* result) { return modbus_exception_code::illegal_function; }
        virtual modbus_exception_code::modbus_exception_code write_single_coil(uint16_t address, bool value) { uint8_t tmp = value ? 1 : 0; return write_multiple_coils(address, 1, &tmp); }
        virtual modbus_exception_code::modbus_exception_code write_single_register(uint16_t address, uint16_t value) { return write_multiple_registers(address, 1, &value); }
        virtual modbus_exception_code::modbus_exception_code write_multiple_registers(uint16_t address, uint16_t count, const uint16_t* values) { return modbus_exception_code::illegal_function; }
        virtual modbus_exception_code::modbus_exception_code write_multiple_coils(uint16_t address, uint16_t count, const uint8_t* values) { return modbus_exception_code::illegal_function; }
    };
}
#endif