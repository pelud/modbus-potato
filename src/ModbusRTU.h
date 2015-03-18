#include "ModbusInterface.h"
namespace ModbusPotato
{
    /// <summary>
    /// This class handles the RTU based protocol for Modbus.
    /// </summary>
    /// <remarks>
    /// See the IFramer interface for a complete description of the public
    /// methods.
    ///
    /// The setup() method must be called with the correct baud rate before
    /// using this class in order to calculate the proper inter-character and
    /// inter-frame delays.
    /// </remarks>
    class CModbusRTU : public IFramer
    {
    public:
        /// <summary>
        /// Constructor for the RTU framer.
        /// </summary>
        CModbusRTU(IStream* stream, ITimeProvider* timer);

        /// <summary>
        /// Initialize any special values.
        /// </summary>
        /// <remarks>
        /// Notice that this method does NOT setup the serial link (i.e.
        /// Serial.begin(...)).  The baud rate is only needed to calculate
        /// the inter-character delays.
        /// </remarks>
        void setup(unsigned long baud);
        virtual void set_handler(IFrameHandler* handler) { m_handler = handler; }
        virtual uint8_t station_address() const { return m_station_address; }
        virtual void set_station_address(uint8_t address) { m_station_address = address; }
        virtual unsigned long poll();
        virtual bool begin_send();
        virtual void send();
        virtual void finished();
        virtual bool frame_ready() const { return m_state == state_frame_ready; }
        virtual uint8_t frame_address() const { return m_frame_address; }
        virtual void set_frame_address(uint8_t address) { m_frame_address = address; }
        virtual uint8_t* buffer() { return m_buffer; }
        virtual size_t buffer_len() const { return m_buffer_len; }
        virtual void set_buffer_len(size_t len) { m_buffer_len = len; }
        virtual size_t buffer_max() const { return MAX_BUFFER - CRC_LEN; }
    private:
        enum
        {
            CRC_LEN = 2,
            MAX_BUFFER = 256 - 1,
            default_baud_rate = 19200,
            default_3t5_period = 1750, // T3.5 character timeout for high baud rates, in microseconds
            default_1t5_period = 750, // T1.5 character timeout for high baud rates, in microseconds
            minimum_tick_count = 2,
            quantization_rounding_count = 2,
            min_pdu_length = 3, // minimum PDU length, excluding the station address. function code and two crc bytes
        };
        IStream* m_stream;
        ITimeProvider* m_timer;
        IFrameHandler* m_handler;
        size_t m_buffer_len;
        uint8_t m_buffer[MAX_BUFFER];
        uint16_t m_crc;
        uint8_t m_station_address, m_frame_address;
        uint8_t m_buffer_tx_pos;
        enum state_type
        {
            state_dump,
            state_idle,
            state_receive,
            state_frame_ready,
            state_queue,
            state_tx_addr,
            state_tx_pdu,
            state_tx_crc,
            state_tx_wait,
            state_collision,
            state_exception
        };
        state_type m_state;
        system_tick_t m_last_ticks;
        system_tick_t m_T3p5, m_T1p5;
    };
}