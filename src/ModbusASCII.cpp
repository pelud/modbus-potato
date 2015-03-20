#include "ModbusASCII.h"
#ifdef _MSC_VER
#undef max
#endif
#define ISXDIGIT(ch) (((ch) >= '0' && (ch) <= '9') || ((ch) >= 'A' && (ch) <= 'F') || ((ch) >= 'a' && (ch) <= 'f'))
#define ASC2BIN(ch) ((ch) <= '9' ? (ch) - '0' : ((ch) | 0x20) - 'a' + 10)
#define BIN2ASC(n) ((n) <= 9 ? (char)((n) + '0') : (char)((n) - 10 + 'A'))
namespace ModbusPotato
{
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

    CModbusASCII::CModbusASCII(IStream* stream, ITimeProvider* timer, uint8_t* buffer, size_t buffer_max)
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
        ,   m_state(state_idle)
        ,   m_last_ticks()
        ,   m_T1s()
    {
        if (!m_stream || !m_timer || !m_buffer || m_buffer_max < 3)
        {
            m_state = state_exception;
            return;
        }

        // set the default timeout
        set_timeout(default_timeout);

        // update the system tick count
        m_last_ticks = m_timer->ticks();
    }

    void CModbusASCII::set_timeout(unsigned int milliseconds)
    {
        m_T1s = 1000000 / m_timer->microseconds_per_tick();
    }

    unsigned long CModbusASCII::poll()
    {
        // state machine for handling incoming data
        //
        // See http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
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
        case state_idle: // waiting for something to happen
idle:       
            {
                while (int ec = m_stream->read(&m_frame_address, 1))
                {
                    // make sure the character was read properly and that it's the start of frame
                    if (ec > 0 && m_frame_address == ':')
                    {
                        // if so, go to the ascii rx address high state
                        m_state = state_rx_addr_high;
                        m_last_ticks = m_timer->ticks();
                        goto rx_addr;
                    }
                }
                return 0; // waiting for an event
            }
        case state_frame_ready: // waiting for the application layer to process the frame
        case state_queue: // waiting for the application layer to create frame for transmission
        case state_collision: // bus collision
            {
                // check for timeout or collisions
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
                }
                return 0; // waiting for user
            }
        case state_rx_addr_high: // receiving the high or low byte of the slave address [ASCII]
        case state_rx_addr_low:
rx_addr:
            {
                // check how much time has elapsed
                system_tick_t elapsed = ELAPSED(m_last_ticks, m_timer->ticks());
                if (elapsed > m_T1s)
                {
                    // timeout, go to the idle state
                    m_state = state_idle;
                    goto idle; // enter the 'idle' state
                }

                // attempt to read the next character
                uint8_t ch;
                int result = m_stream->read(&ch, 1);
                if (result < 0)
                {
                    // read error, go to the idle state
                    m_state = state_idle;
                    goto idle; // enter the 'idle' state
                }

                // check if anything was done
                if (!result)
                    return m_T1s - elapsed; // wait for the timeout

                // check if we got the start of frame character
                if (ch == ':')
                {
                    // if so, start over and go back to the rx_addr_high state
                    m_state = state_rx_addr_high;
                    m_last_ticks = m_timer->ticks();
                    goto rx_addr;
                }

                // make sure the character is valid
                if (!ISXDIGIT(ch))
                {
                    // invalid character, go to the idle state
                    m_state = state_idle;
                    goto idle; // enter the 'idle' state
                }

                // convert the character from ascii to binary
                ch = ASC2BIN(ch);

                // check if we have read the low nibble yet
                if (m_state == state_rx_addr_high)
                {
                    // if not, read low nibble state
                    m_frame_address = ch;
                    m_state = state_rx_addr_low;
                    m_last_ticks = m_timer->ticks();
                    goto rx_addr;
                }

                // shift the low nibble into the frame address
                m_frame_address <<= 4;
                m_frame_address |= ch;

                // check to see if the frame address matches our station address
                if (m_station_address && m_frame_address && m_station_address != m_frame_address)
                {
                    // no match, go back to the idle state
                    m_state = state_idle;
                    goto idle;
                }

                // initialize the checksum and the data buffer
                m_checksum = m_frame_address;
                m_buffer_len = 0;
                m_state = state_rx_pdu_high;
                m_last_ticks = m_timer->ticks();
                goto rx_pdu;
            }
        case state_rx_pdu_high: // receiving the high or low byte of the PDU [ASCII]
        case state_rx_pdu_low:
rx_pdu:
            {
                system_tick_t now = m_timer->ticks();
                for (;;)
                {
                    // check how much time has elapsed
                    system_tick_t elapsed = ELAPSED(m_last_ticks, now);
                    if (elapsed > m_T1s)
                    {
                        // timeout, go to the idle state
                        m_state = state_idle;
                        goto idle; // enter the 'idle' state
                    }

                    // attempt to read the next character
                    uint8_t ch;
                    int result = m_stream->read(&ch, 1);
                    if (result < 0)
                    {
                        // read error, go to the idle state
                        m_state = state_idle;
                        goto idle; // enter the 'idle' state
                    }

                    // check if anything was done
                    if (!result)
                        return m_T1s - elapsed; // wait for the timeout

                    // check if we got the start of frame character
                    if (ch == ':')
                    {
                        // if so, start over and go back to the rx_addr_high state
                        m_state = state_rx_addr_high;
                        m_last_ticks = now;
                        goto rx_addr;
                    }

                    // check if we reached the end of the message
                    if (ch == '\r')
                    {
                        // make sure we are not half way through a nibble
                        if (m_state != state_rx_pdu_high)
                        {
                            // if so, drop the packet and go back to the 'idle' state
                            m_state = state_idle;
                            goto idle;
                        }

                        // got carriage return, wait for the final line feed
                        m_state = state_rx_cr;
                        m_last_ticks = m_timer->ticks();
                        goto rx_cr;
                    }

                    // make sure the character is valid and that we have not over-run the end of the buffer
                    if (!ISXDIGIT(ch) || m_buffer_len == m_buffer_max)
                    {
                        // invalid character or too many characters, go to the idle state
                        m_state = state_idle;
                        goto idle; // enter the 'idle' state
                    }

                    // convert the character from ascii to binary
                    ch = ASC2BIN(ch);

                    // check if we have read the low nibble yet
                    if (m_state == state_rx_pdu_high)
                    {
                        // if not, go to the read low nibble state
                        m_buffer[m_buffer_len] = ch;
                        m_state = state_rx_pdu_low;
                        m_last_ticks = now;
                        continue;
                    }

                    // shift the low nibble into the data buffer
                    uint8_t& bufp = m_buffer[m_buffer_len];
                    bufp <<= 4;
                    bufp |= ch;

                    // update the checksum and move to the next character
                    m_checksum = (uint8_t)(m_checksum + bufp);
                    m_buffer_len++;
                    m_state = state_rx_pdu_high;
                    m_last_ticks = now;
                    continue;
                }
            }
        case state_rx_cr: // got carriage return, waiting for final line feed
rx_cr:
            {
                // check how much time has elapsed
                system_tick_t elapsed = ELAPSED(m_last_ticks, m_timer->ticks());
                if (elapsed > m_T1s)
                {
                    // timeout, go to the idle state
                    m_state = state_idle;
                    goto idle; // enter the 'idle' state
                }

                // attempt to read the next character
                uint8_t ch;
                int result = m_stream->read(&ch, 1);
                if (result < 0)
                {
                    // read error, go to the idle state
                    m_state = state_idle;
                    goto idle; // enter the 'idle' state
                }

                // check if anything was done
                if (!result)
                    return m_T1s - elapsed; // wait for the timeout

                // check if we got the start of frame character
                if (ch == ':')
                {
                    // if so, start over and go back to the rx_addr_high state
                    m_state = state_rx_addr_high;
                    m_last_ticks = m_timer->ticks();
                    goto rx_addr;
                }

                // make sure we got the line feed and that the checksum is correct
                if (ch != '\n' && m_buffer_len >= min_pdu_length && m_checksum == 0)
                {
                    // if not, drop the packet and go back to the 'idle' state
                    m_state = state_idle;
                    goto idle; // enter the 'idle' state
                }

                // LRC passed, remove the LRC byte
                m_buffer_len -= LRC_LEN;

                // move to the 'Frame Ready' state
                m_state = state_frame_ready;
                m_last_ticks = m_timer->ticks();

                // execute the callback
                if (m_handler)
                    m_handler->frame_ready(this);

                // evaluate the switch statement again in case something has changed
                return poll(); // jump to the start of the function to re-evalutate entire switch statement
            }
        case state_tx_sof: // transmitting start of frame character [ASCII]
            {
                // try and write the start of frame character
                uint8_t ch = ':';
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // SOF; move to the 'TX ADDR HIGH' state
                    m_state = state_tx_addr_high;
                    goto tx_addr_high;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_addr_high: // transmitting remote station address [ASCII]
tx_addr_high:
            {
                // convert the high nibble of the station address to ASCII HEX
                uint8_t ch = m_frame_address >> 4;
                ch = BIN2ASC(ch);

                // try and write the remote station address
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // high nibble of address sent; now send the low nibble
                    m_checksum = m_frame_address;
                    m_state = state_tx_addr_low;
                    goto tx_addr_low;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_addr_low: // transmitting remote station address [ASCII]
tx_addr_low:
            {
                // convert the low nibble of the station address to ASCII HEX
                uint8_t ch = m_frame_address & 0xf;
                ch = BIN2ASC(ch);

                // try and write the remote station address
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // low nibble of address sent; now send the high nibble of the first PDU byte
                    m_state = state_tx_pdu_high;
                    m_buffer_tx_pos = 0;
                    goto tx_pdu_high;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_pdu_high: // transmitting PDU [ASCII]
tx_pdu_high:
            {
                // convert the high nibble of the next PDU byte to ASCII HEX
                uint8_t ch = m_buffer[m_buffer_tx_pos] >> 4;
                ch = BIN2ASC(ch);

                // try and write the remote station address
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // high nibble of address sent; update checksum and send the low nibble
                    m_checksum = (uint8_t)(m_checksum + m_buffer[m_buffer_tx_pos]);
                    m_state = state_tx_pdu_low;
                    goto tx_pdu_low;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_pdu_low: // transmitting PDU [ASCII]
tx_pdu_low:
            {
                // convert the low nibble of the next PDU byte to ASCII HEX
                uint8_t ch = m_buffer[m_buffer_tx_pos] & 0xf;
                ch = BIN2ASC(ch);

                // try and write the remote station address
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // low nibble transmitted; move to the next byte in the PDU buffer
                    m_buffer_tx_pos++;

                    // check if we are finished
                    if (m_buffer_tx_pos == m_buffer_len)
                    {
                        // negate the checksum (2's complement) so that everything will add to 0 at the receiving end
                        m_checksum = (uint8_t)-(int8_t)m_checksum;

                        // finished sending the PDU; now send the LRC high nibble
                        m_state = state_tx_lrc_high;
                        goto tx_lrc_high;
                    }

                    // low nibble of address sent; now send the high nibble of the next PDU byte
                    m_state = state_tx_pdu_high;
                    goto tx_pdu_high;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_lrc_high: // transmitting LRC high [ASCII]
tx_lrc_high:
            {
                // convert the high nibble of the LRC to ASCII HEX
                uint8_t ch = m_checksum >> 4;
                ch = BIN2ASC(ch);

                // try and write the value
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // high nibble sent; now send the low nibble
                    m_state = state_tx_lrc_low;
                    goto tx_lrc_low;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_lrc_low: // transmitting LRC low [ASCII]
tx_lrc_low:
            {
                // convert the high nibble of the LRC to ASCII HEX
                uint8_t ch = m_checksum & 0xf;
                ch = BIN2ASC(ch);

                // try and write the value
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // low nibble sent; now send the cr
                    m_state = state_tx_cr;
                    goto tx_cr;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_cr: // transmitting carriage return character [ASCII]
tx_cr:
            {
                // try and write the start of frame character
                uint8_t ch = '\r';
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // done; move to the line feed character
                    m_state = state_tx_lf;
                    goto tx_lf;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_lf: // transmitting carriage return character [ASCII]
tx_lf:
            {
                // try and write the start of frame character
                uint8_t ch = '\n';
                if (int ec = m_stream->write(&ch, 1))
                {
                    // check if something bad happened
                    if (ec < 0)
                    {
                        m_state = state_exception;
                        return 0; // fatal exception
                    }

                    // done; move to the line feed character
                    m_state = state_tx_wait;
                    goto tx_wait;
                }

                return 0; // waiting for room in the write buffer
            }
        case state_tx_wait: // waiting for the characters to finish transmitting [ASCII]
tx_wait:
            {
                // dump our own echo
                m_stream->read(NULL, (size_t)-1);

                // poll if the write has completed
                if (m_stream->writeComplete())
                {
                    // transmission complete; disable the RS-485 transmitter
                    m_stream->txEnable(false);

                    // done! go to the idle state
                    m_state = state_idle;
                    goto idle;
                }

                return 0; // waiting for write buffer to drain
            }
       }

        // if we get here, then something terrible has happened such as memory corruption
        m_state = state_exception;
        return 0;
    }

    bool CModbusASCII::begin_send()
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

    void CModbusASCII::send()
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
                // enter the transmit start of frame state
                m_state = state_tx_sof;

                // enable the transmitter
                m_stream->txEnable(true);
                return; // ok -- we expect that the user must call poll() at this point.
            }
        case state_collision: // bus collision
            {
                // abort the response and go to the idle state
                m_state = state_idle;
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

    void CModbusASCII::finished()
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
                m_state = state_idle;
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
