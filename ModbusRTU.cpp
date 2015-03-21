#include "ModbusRTU.h"
#ifdef _MSC_VER
#undef max
#endif
namespace ModbusPotato
{
    // calculate the inter-character delay (T3.5 and T1.5) values
    #define CALC_INTER_CHAR_DELAY(f, baud) ( (f) * 11 / (baud) )
    #ifdef _MSC_VER
    static_assert(CALC_INTER_CHAR_DELAY(3500000, 9600) == 4010, "invalid intercharacter delay calculation");
    static_assert(CALC_INTER_CHAR_DELAY(3500000, 300) == 128333, "invalid intercharacter delay calculation");
    #endif

    // calculate the amount of time elapsed
    //
    // Note: As long as all types are unsigned, and the timer value rolls
    // over at the maximum value of the corresponding data type, this
    // calculation will return the correct result when it rolls over.
    //
    // For example, if m_last_ticks is at 0xffffffff, and system ticks
    // rolls over to 0, the value will be 0 - 0xffffffff, which is the same
    // as 0 - (-1), or 1.
    //
    #define ELAPSED(start, end) ((system_tick_t)(end) - (system_tick_t)(start))
    #ifdef _MSC_VER
    static_assert(~(system_tick_t)0 > 0, "system_tick_t must be unsigned");
    static_assert((system_tick_t)-1 == ~(system_tick_t)0, "two's complement arithmetic required");
    static_assert(ELAPSED(~(system_tick_t)0, 0) == 1, "elapsed time roll-over check failed");
    #endif

    // accumulate the next byte of the CRC
    enum { POLY = 0xa001 };
    static inline uint16_t crc16_modbus(uint16_t crc, const uint8_t* buffer, size_t len)
    {
        for (; len; buffer++, len--)
        {
            crc ^= *buffer;
            crc = (crc & 1) != 0 ? ((crc >> 1) ^ POLY) : (crc >> 1);
            crc = (crc & 1) != 0 ? ((crc >> 1) ^ POLY) : (crc >> 1);
            crc = (crc & 1) != 0 ? ((crc >> 1) ^ POLY) : (crc >> 1);
            crc = (crc & 1) != 0 ? ((crc >> 1) ^ POLY) : (crc >> 1);
            crc = (crc & 1) != 0 ? ((crc >> 1) ^ POLY) : (crc >> 1);
            crc = (crc & 1) != 0 ? ((crc >> 1) ^ POLY) : (crc >> 1);
            crc = (crc & 1) != 0 ? ((crc >> 1) ^ POLY) : (crc >> 1);
            crc = (crc & 1) != 0 ? ((crc >> 1) ^ POLY) : (crc >> 1);
        }
        return crc;
    }

    CModbusRTU::CModbusRTU(IStream* stream, ITimeProvider* timer, uint8_t* buffer, size_t buffer_max)
        :   m_stream(stream)
        ,   m_timer(timer)
        ,   m_buffer(buffer)
        ,   m_buffer_max(buffer_max)
        ,   m_buffer_len()
        ,   m_handler()
        ,   m_checksum()
        ,   m_station_address()
        ,   m_frame_address()
        ,   m_buffer_tx_pos()
        ,   m_state(state_dump)
        ,   m_last_ticks()
        ,   m_T3p5()
        ,   m_T1p5()
    {
        if (!m_stream || !m_timer || !m_buffer || m_buffer_max < 3)
        {
            m_state = state_exception;
            return;
        }

        // put some default values into the delays
        setup(default_baud_rate);

        // update the system tick count
        m_last_ticks = m_timer->ticks();
    }

    void CModbusRTU::setup(unsigned long baud)
    {
        // calculate the intercharacter delays in microseconds
        unsigned int t3p5 = default_3t5_period;
        unsigned int t1p5 = default_1t5_period;
        if (baud && baud <= 19200)
        {
            t3p5 = CALC_INTER_CHAR_DELAY(3500000, baud);
            t1p5 = CALC_INTER_CHAR_DELAY(1500000, baud);
        }

        // convert the intercharacter delays from microseconds to system ticks
        //
        // Note: on systems that have poor resolution timers, we must round
        // down and wait the minimum time quanta when waiting for the end of
        // the packet timeout when receiving packets.  When transmitting, we
        // must round up and wait the full time quanta before we can transmit
        // again to ensure that consequitive broadcast packets from a master
        // are not dropped by the slaves.
        //
        // For example, if the timer resolution is 1ms, the delay for 3.5
        // characters at 9600 baud should be 4.01ms, which rounds down to 4
        // counts when waiting for for others.  Due to the quantization error
        // of the timer, after 4 counts have passed on the timer, the actual
        // delay waited will be between 3ms (if the start time was latched at
        // the end of the time period) and 4ms (if the start time was latched
        // at the start of the time period.  
        //
        m_T3p5 = t3p5 / m_timer->microseconds_per_tick();
        m_T1p5 = t1p5 / m_timer->microseconds_per_tick();

        // make sure the delays are each at least 2 counts
        m_T3p5 = m_T3p5 < minimum_tick_count ? minimum_tick_count : m_T3p5;
        m_T1p5 = m_T1p5 < minimum_tick_count ? minimum_tick_count : m_T1p5;
    }

    unsigned long CModbusRTU::poll()
    {
        // state machine for handling incoming data
        //
        //                    -------------           -------------
        //        +--Sent--->|   TX CRC    |    +--->|   TX Wait   |
        //        |           -------------     |     -------------
        //        |                 |           |           |
        //  -------------         Sent      TX Empty      T3.5                  start
        // |   TX PDU    |          |           |           |                     |
        //  -------------           v           |           |                     v
        //        ^           -------------     |           v               -------------
        //        |          |  TX Drain   |----+    +------+<----T3.5-----|    Dump     |
        //      Sent          -------------          |                      ------------- 
        //        |                                  v                            ^
        //  -------------                      -------------                      |
        // |   TX Addr   |   +--begin_send()--|    Idle     |----Invalid Char---->+
        //  -------------    |                 -------------                      ^
        //        ^          |                   ^       |                        |
        //        |      +---+                   |  Addr. Match                   |
        //      send()   |            +----------+       |                        |
        //        |      v            |                  v                        |
        //     -------------  fini-   |   T3.5+  -------------                    |
        //    |    Queue    |-shed()->+<--CRC/--|   Receive   |---T1.5/Comm Err-->+
        //     -------------          ^   F.E.   -------------                    ^
        //       |      ^             |                |                          |
        //       |      |             |          T3.5+Frame OK           finished()/send()
        //       | begin_send()   finished()           |                          |
        //       |      |             |                v                          |
        //       |      |             |          -------------              ------------- 
        //       |      +-------------+---------| Frame Ready |--Receive-->|  Collision  |
        //       |                               -------------              -------------
        //    Receive                                                             ^
        //       |                                                                |
        //       +----------------------------------------------------------------+
        //
        // See http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
        //
        // This state machine is based on Figure 14 of the above PDF with the
        // "Control and Waiting" state split into "Dump", "Frame Ready" and
        // "Queue", and the "Emission" state split into "TX Addr", "TX PDU" and
        // "TX CRC" and "TX Done".
        //
        // Reason for goto statements: re-evaluate switch case labels when
        // changing states.
        //
        switch (m_state)
        {
        case state_exception: // fatal error - framer shut down
            {
                // do nothing
                return 0;
            }
        case state_dump: // dump any unwanted incoming data
dump:
            {
                // if not, check how much time has elapsed
                system_tick_t elapsed = ELAPSED(m_last_ticks, m_timer->ticks());

                // if the timer is done, then go to the idle state
                if (elapsed >= m_T3p5)
                {
                    m_state = state_idle;
                    m_stream->communicationStatus(false, false);
                    goto idle; // waiting for an event
                }

                // dump any remaining data
                if (m_stream->read(NULL, (size_t)-1))
                {
                    // reset the T3.5 timer
                    m_last_ticks = m_timer->ticks();
                    return m_T3p5; // waiting for T3.5 timer
                }

                // if the timer has not finished, then return the amount of time remaining
                return m_T3p5 - elapsed;
            }
        case state_idle: // waiting for something to happen
idle:       
            {
                if (int ec = m_stream->read(&m_frame_address, 1))
                {
                    // make sure the character is valid
                    if (ec < 0 || (m_frame_address && m_station_address && m_frame_address != m_station_address))
                    {
                        // invalid character received - reset the timer and enter the 'dump' state.
                        m_last_ticks = m_timer->ticks();
                        m_state = state_dump;
                        m_stream->communicationStatus(true, false);
                        goto dump; // enter the dump state
                    }

                    // initialize the CRC and accumulate the frame address
                    m_checksum = crc16_modbus(0xffff, &m_frame_address, 1);

                    // broadcast or station address match, enter the receiving state
                    m_state = state_receive;
                    m_buffer_len = 0;
                    m_last_ticks = m_timer->ticks();
                    m_stream->communicationStatus(true, false);
                    goto receive; // enter the receive state
                }
                return 0; // waiting for an event
            }
        case state_frame_ready: // waiting for the application layer to process the frame
        case state_queue: // waiting for the application layer to create frame for transmission
            {
                // check for collisions
                //
                // If this happens in the frame_ready state then it means that
                // the master probably thinks that the slave timed out and is
                // re-transmitting, or there are multiple masters or slaves
                // with the same address.
                //
                if (m_stream->read(NULL, (size_t)-1))
                {
                    m_state = state_collision;
                    m_last_ticks = m_timer->ticks();
                    m_stream->communicationStatus(true, false);
                }
                return 0; // waiting for user
            }
        case state_collision: // bus collision
            {
                return 0; // waiting for user
            }
        case state_receive: // actively receiving new data
receive:
            {
                // check how much time has elapsed
                system_tick_t elapsed = ELAPSED(m_last_ticks, m_timer->ticks());

                // check if there are any waiting characters
                if (int ec = m_stream->read(m_buffer + m_buffer_len, m_buffer_max - m_buffer_len))
                {
                    // check for comm errors or if the inter-character delay has been exceeded
                    //
                    // Note: we must add two to the timer to account for
                    // rounding and quantization error.
                    //
                    if (ec < 0 || elapsed >= (m_T1p5 + quantization_rounding_count))
                    {
                        // if so, reset the timer and enter the 'dump' state.
                        m_last_ticks = m_timer->ticks();
                        m_state = state_dump;
                        goto dump; // enter the dump state
                    }

                    // update the CRC and advance the buffer pointer
                    m_checksum = crc16_modbus(m_checksum, m_buffer + m_buffer_len, ec);
                    m_buffer_len += ec;

                    // reset the timer
                    m_last_ticks = m_timer->ticks();
                    elapsed = 0;
                }

                // check if there is still input even after we have filled the buffer
                if (m_buffer_max == m_buffer_len && m_stream->read(NULL, (size_t)-1))
                {
                    // if so, reset the timer and enter the 'dump' state.
                    m_last_ticks = m_timer->ticks();
                    m_state = state_dump;
                    goto dump; // enter the dump state
                }

                // check if the T3.5 timer has elapsed
                if (elapsed < m_T3p5)
                    return m_T3p5 - elapsed; // wait for the timer to elapse

                // check the CRC
                //
                // Note: They did the CRC properly in Modbus, so all we
                // have to do is make sure the CRC value is 0 after the two
                // CRC bytes from the frame have been accumulated.  We
                // don't really need the length check as it's unlikely for
                // the crc to be 0 without receiving the check bytes, but
                // it doesn't hurt to have it.
                //
                if (m_buffer_len < min_pdu_length || m_checksum != 0)
                {
                    // if the CRC failed, then dump the frame and go back to idle
                    m_last_ticks = m_timer->ticks();
                    m_state = state_idle;
                    m_stream->communicationStatus(false, false);
                    goto idle; // enter the idle state
                }

                // crc passed, remove the two CRC bytes
                m_buffer_len -= CRC_LEN;

                // move to the 'Frame Ready' state
                m_state = state_frame_ready;
                m_last_ticks = m_timer->ticks();
                m_stream->communicationStatus(false, false);

                // execute the callback
                if (m_handler)
                    m_handler->frame_ready(this);

                // evaluate the switch statement again in case something has changed
                return poll(); // jump to the start of the function to re-evalutate entire switch statement
            }
        case state_tx_addr: // transmitting remote station address [RTU]
            {
                // dump any incoming data
                //
                // This should not happen and if it does then it's probably
                // a bus collision so we abort the transmission.  Also check
                // your flow control settings on both the sending and receiving
                // side.
                //
                if (m_stream->read(NULL, (size_t)-1))
                {
                    // reset the timer and go to the dump state
                    m_last_ticks = m_timer->ticks();
                    m_state = state_dump;
                    m_stream->communicationStatus(true, false);
                    goto dump; // dump any remaining data
                }

                // Wait for the full time delay.
                //
                // We must wait an additional two counts after the last T3.5 to
                // ensure that we have waited the full timeout due to rounding
                // error as well as the quantization error of the system clock.
                //
                // Please be aware that this assumes that the system clock will
                // not roll over between calls to send().  However, even if it
                // does, the worse case will be an additional T3.5 + 2 count
                // delay.
                //
                system_tick_t elapsed = ELAPSED(m_last_ticks, m_timer->ticks());
                if (elapsed < (m_T3p5 + quantization_rounding_count))
                    return m_T3p5 + quantization_rounding_count - elapsed; // waiting to send

                // try and write the remote station address
                if (int ec = m_stream->write(&m_frame_address, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        m_stream->communicationStatus(false, false);
                        return 0; // fatal exception
                    }

                    // address sent; update the CRC while we send the frame address and move to the 'TX PDU' state
                    m_checksum = crc16_modbus(0xffff, &m_frame_address, 1);
                    m_state = state_tx_pdu;
                    m_buffer_tx_pos = 0;
                    goto tx_pdu;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_pdu: // transmitting frame PDU
tx_pdu:
            {
                // send the next chunk
                if (int ec = m_stream->write(m_buffer + m_buffer_tx_pos, m_buffer_len - m_buffer_tx_pos))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        m_stream->communicationStatus(false, false);
                        return 0; // fatal exception
                    }

                    // update the CRC while we send the bytes and advance the buffer tx position
                    m_checksum = crc16_modbus(m_checksum, m_buffer + m_buffer_tx_pos, ec);
                    m_buffer_tx_pos += ec;
                }

                // dump our own echo
                m_stream->read(NULL, (size_t)-1);

                // check if we should start sending the CRC
                if (m_buffer_tx_pos == m_buffer_len)
                {
                    // if so, enter the 'TX CRC' state
                    m_state = state_tx_crc;
                    m_buffer_tx_pos = 0;
                    goto tx_crc; // enter the 'TX CRC' state
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_crc: // transmitting the frame CRC
tx_crc:
            {
                // similiar to above, except we send the CRC instead of the data
                while (m_buffer_tx_pos != CRC_LEN)
                {
                    // write the next byte in the CRC
                    uint8_t ch = (uint8_t)m_checksum;
                    int ec = m_stream->write(&ch, 1);
                    if (!ec)
                        break;

                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        m_stream->communicationStatus(false, false);
                        return 0; // fatal exception
                    }

                    // advance the high byte of the CRC to the low byte and start again
                    m_checksum >>= 8;
                    m_buffer_tx_pos++;
                }

                // dump our own echo
                m_stream->read(NULL, (size_t)-1);

                // check if we should enter the 'TX Drain' state
                if (m_buffer_tx_pos == CRC_LEN)
                {
                    m_state = state_tx_drain;
                    goto tx_drain; // enter the 'TX Drain' state
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_drain: // waiting for the characters to finish transmitting
tx_drain:
            {
                // dump our own echo
                m_stream->read(NULL, (size_t)-1);

                // poll if the write has completed
                if (m_stream->writeComplete())
                {
                    // transmission complete; disable the RS-485 transmitter
                    m_stream->txEnable(false);

                    // go to the tx wait state so we can wait for the T3.5 delay
                    m_last_ticks = m_timer->ticks();
                    m_state = state_tx_wait;
                    m_stream->communicationStatus(false, false);
                    goto tx_wait;
                }

                return 0; // waiting for write buffer to drain
            }
        case state_tx_wait: // waiting for final T3.5 delay after transmitting
tx_wait:
            {
                // dump our own echo
                m_stream->read(NULL, (size_t)-1);

                // check if the T3.5 timer has elapsed
                system_tick_t elapsed = ELAPSED(m_last_ticks, m_timer->ticks());
                if (elapsed < m_T3p5)
                    return m_T3p5 - elapsed; // wait for the timer to elapse

                // TX done! go to the idle state
                m_state = state_idle;
                goto idle;
            }
        }

        // if we get here, then something terrible has happened such as memory corruption
        m_state = state_exception;
        return 0;
    }

    bool CModbusRTU::begin_send()
    {
        switch (m_state)
        {
        case state_collision:
            {
                return true; // if there was a collision then return true so that the user will call send() or finished()
            }
        case state_queue:
            {
                return true; // already in the queue state
            }
        case state_idle:
        case state_frame_ready:
            {
                m_state = state_queue; // set the state machine to the 'queue' state we the user can access the buffer
                return true;
            }
        }
        return false; // not ready to send
    }

    void CModbusRTU::send()
    {
        // sanity check
        if (m_buffer_len >= buffer_max())
        {
            // buffer overflow - enter the 'exception' state
            m_state = state_exception;
            return;
        }

        switch (m_state)
        {
        case state_queue: // buffer is ready
            {
                // enter the transmit station address state
                m_state = state_tx_addr;
                m_stream->communicationStatus(false, true);

                // enable the transmitter
                m_stream->txEnable(true);
                return; // ok -- we expect that the user must call poll() at this point.
            }
        case state_collision: // bus collision
            {
                // abort the response and go to the dump state
                //
                // Note: The timer should be set already when entering the
                // collision state.
                //
                m_state = state_dump;
                return; // collision, abort transmission and dump any further incoming data
            }
        default:
            {
                // invalid state, user probably didn't call begin_send()
                m_state = state_exception;
                return; // invalid state - enter the 'exception' state
            }
        }
    }

    void CModbusRTU::finished()
    {
        switch (m_state)
        {
        case state_frame_ready: // received
        case state_queue: // aborting begin_send()
            {
                // acknowledge or abort the user lock on the buffer
                m_state = state_idle;
                return; // ok
            }
        case state_collision: // bus collision
            {
                // more data started when we were not expecting it
                m_state = state_dump;
                return; // collision, dump any further incoming data
            }
        default:
            {
                // invalid state
                m_state = state_exception;
                return; // invalid state - enter the 'exception' state
            }
        }
    }
}
