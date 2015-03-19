#include "ModbusInterface.h"
namespace ModbusPotato
{
    /// <summary>
    /// This class implements a basic Modbus slave interface.
    /// </summary>
    class CModbusSlave : public IFrameHandler
    {
    public:
        CModbusSlave(IFramer* framer, ISlaveHandler* handler);
        virtual void frame_ready();
    private:
        uint8_t read_bit_input_rsp(bool discrete);
        uint8_t read_registers_rsp(bool holding);
        uint8_t write_single_coil_rsp();
        uint8_t write_single_register_rsp();
        uint8_t write_multiple_coils_rsp();
        uint8_t write_multiple_registers_rsp();
        IFramer* m_framer;
        ISlaveHandler* m_handler;
        enum
        {
            read_coil_status = 0x01,
            read_discrete_input_status = 0x02,
            read_holding_registers = 0x03,
            read_input_registers = 0x04,
            write_single_coil = 0x05,
            write_single_register = 0x06,
            write_multiple_coils = 0x0f,
            write_multiple_registers = 0x10,
        };
    };
}
