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
        IFramer* m_framer;
        ISlaveHandler* m_handler;
        enum
        {
            read_holding_registers = 0x03,
            preset_single_register = 0x06,
            write_multiple_registers = 0x10,
        };
    };
}
