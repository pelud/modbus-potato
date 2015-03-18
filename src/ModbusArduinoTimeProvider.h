#include "ModbusInterface.h"
namespace ModbusPotato
{
    /// <summary>
    /// This class providers a microsecond level clock for the Arduino.
    /// </summary>
    class CModbusArduinoTimeProvider : public ITimeProvider
    {
        virtual ModbusPotato::system_tick_t ticks() const
        {
            return micros();
        }
        virtual unsigned long microseconds_per_tick() const
        {
            return 1;
        }
    };  
}
