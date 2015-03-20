#ifndef __ModbusPotato_ModbusASCII_h__
#define __ModbusPotato_ModbusASCII_h__
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
    class CModbusASCII : public IFramer
    {
    public:
        /// <summary>
        /// Constructor for the RTU framer.
        /// </summary>
        CModbusASCII(IStream* stream, ITimeProvider* timer, uint8_t* buffer, size_t buffer_max);

        /// <summary>
        /// Sets the timeout, in milliseconds
        /// </summary>
        /// <remarks>
        /// The poll() method must be called and the timer adjusted according
        /// to the semantics described in IFramer::poll() for the new timeout
        /// to take effect.
        /// </remarks>
        void set_timeout(unsigned int milliseconds);

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
        virtual size_t buffer_max() const { return m_buffer_max; }
    private:
        enum
        {
            LRC_LEN = 1,
            min_pdu_length = 2, // minimum PDU length, excluding the station address. function code and one LRC byte
            default_timeout = 1000, // default timeout, in milliseconds
        };
        IStream* m_stream;
        ITimeProvider* m_timer;
        IFrameHandler* m_handler;
        uint8_t* m_buffer;
        size_t m_buffer_len, m_buffer_max;
        uint8_t m_checksum;
        uint8_t m_station_address, m_frame_address;
        uint8_t m_buffer_tx_pos;
        enum state_type
        {
            state_exception,
            state_idle,
            state_frame_ready,
            state_queue,
            state_collision,
            state_rx_addr_high,
            state_rx_addr_low,
            state_rx_pdu_high,
            state_rx_pdu_low,
            state_rx_cr,
            state_tx_sof,
            state_tx_addr_high,
            state_tx_addr_low,
            state_tx_pdu_high,
            state_tx_pdu_low,
            state_tx_lrc_high,
            state_tx_lrc_low,
            state_tx_cr,
            state_tx_lf,
            state_tx_wait,
        };
        state_type m_state;
        system_tick_t m_last_ticks;
        system_tick_t m_T1s;
    };
}
#endif