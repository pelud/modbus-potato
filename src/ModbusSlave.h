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
        uint8_t read_registers_rsp(bool holding);
        uint8_t preset_single_register_rsp();
        uint8_t write_multiple_registers_rsp();
        IFramer* m_framer;
        ISlaveHandler* m_handler;
        enum
        {
            read_holding_registers = 0x03,
            read_input_registers = 0x04,
            preset_single_register = 0x06,
            write_multiple_registers = 0x10,
        };
    };
}
