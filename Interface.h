// C++ Interface definitions for Modbus library
//
// See also:
// http://www.modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
// http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf
//
#ifndef __ModbusPotato_Interface_h__
#define __ModbusPotato_Interface_h__
#include <stdint.h>
#ifdef ARDUINO
#include <Arduino.h>
#elif _MSC_VER
#include <Windows.h>
#endif
namespace ModbusPotato
{
#ifdef ARDUINO
typedef unsigned long system_tick_t;
#elif _MSC_VER
typedef DWORD system_tick_t;
#endif

    /// <summary>
    /// Represents an endpoint that can read or write characters.
    /// </summary>
    class IStream
    {
    public:
        virtual ~IStream() {}

        /// <summary>
        /// Read some characters without blocking.
        /// </summary>
        /// <returns>
        /// The number of characters read, or a -1 for a communications error.
        /// </returns>
        /// <remarks>
        /// If there is a parity or framing error in any of the characters,
        /// this function should dump any remaining characters in the input
        /// buffer and return -1.  If no data is in the buffer then this
        /// function must return 0.
        ///
        /// If the buffer pointer is NULL then any characters in the input
        /// buffer must be dumped without storing them.  If any characters are
        /// dumped then this function must return a non-zero value, preferably
        /// the number of characters dumped.
        ///
        /// If the buffer_size parameter is 0, then this function should
        /// immediately return 0.  If the value is (size_t)-1 and the buffer
        /// is NULL then this function must dump all remaining characters.
        /// </remarks>
        virtual int read(uint8_t* buffer, size_t buffer_size) = 0;

        /// <summary>
        /// Sends the given characters without blocking.
        /// </summary>
        /// <returns>
        /// The number of characters written, or -1 if communications exception.
        /// </returns>
        /// <remarks>
        /// This function should only return a negative value in fatal
        /// situations, in which the framer is expected to shut down.
        /// </remarks>
        virtual int write(uint8_t* buffer, size_t len) = 0;

        /// <summary>
        /// Enables or disables the RS-485 transmitter.
        /// </summary>
        /// <remarks>
        /// Ideally the receiver should be disabled when the transmitter is
        /// enabled, but this is not mandatory.
        /// </remarks>
        virtual void txEnable(bool state) = 0;

        /// <summary>
        /// Indicates if all of the characters have been written.
        /// </summary>
        virtual bool writeComplete() = 0;
    };

    /// <summary>
    /// Provides access to the system tick clock.
    /// </summary>
    class ITimeProvider
    {
    public:
        virtual ~ITimeProvider() {}

        /// <summary>
        /// Returns the number of ticks since an arbitrary epoch point
        /// </summary>
        /// <remarks>
        /// The clock must increment monotomically and must roll over at the
        /// maximum size of system_tick_t to 0.  system_tick_t must be an
        /// unsigned integer of an arbitrary size.
        ///
        /// This is in this interface to aid in unit testing.
        /// </remarks>
        virtual system_tick_t ticks() const = 0;

        /// <summary>
        /// Returns the number of microseconds per tick.
        /// </summary>
        /// <remarks>
        /// This is in this interface to aid in unit testing.
        /// </remarks>
        virtual unsigned long microseconds_per_tick() const = 0;
    };

    /// <summary>
    /// This interface implements the framing protocol for Modbus.
    /// </summary>
    class IFramer
    {
    public:
        virtual ~IFramer() {}

        /// <summary>
        /// Returns the station address.
        /// </summary>
        /// <remarks>
        /// If this is 0 then all addresses will be matched.
        /// </remarks>
        virtual uint8_t station_address() const = 0;

        /// <summary>
        /// Sets the station address.
        /// </summary>
        /// <remarks>
        /// If this is not set or is set to 0 then all addresses will be matched.
        /// </remarks>
        virtual void set_station_address(uint8_t address) = 0;

        /// <summary>
        /// Sets the callback to be executed when a frame is ready.
        /// </summary>
        /// <remarks>
        /// This will only be called from the poll() method, and thus will
        /// automatically take care of updating the timers if the send() or
        /// finishied() methods are used within the callback.
        /// </remarks>
        virtual void set_frame_ready_callback(void (*cb)(void* obj), void* obj) = 0;

        /// <summary>
        /// Handles any timeouts and transfers more data as needed.
        /// </summary>
        /// <returns>
        /// The next timeout, in system ticks, or 0 if none.
        /// </returns>
        /// <remarks>
        /// This method can be called repeatedly in the loop() statement.  It
        /// performs all the actual reads and writes to the output device.
        ///
        /// In an interrupt driven system, it must be called after a new
        /// character is available, the transmitter is ready to send more data,
        /// the transmission has completed or after the previously returned
        /// timeout has elapsed.
        ///
        /// This function must be called again after any function call that may
        /// change the state.  Any prior timeout, if still pending, must be
        /// cancelled and replaced with the new one returned by this function.
        /// </remarks>
        virtual unsigned long poll() = 0;

        /// <summary>
        /// Places the state machine into the transmitting state to reserve the
        /// data buffer.
        /// </summary>
        /// <returns>
        /// true if the buffer() is available, false if the state machine is busy.
        /// </returns>
        /// <remarks>
        /// If data reception is already in progress, this method will fail and
        /// return false.
        ///
        /// After a new transmission is started, it must be completed either by
        /// calling the send() method to send the data or the finished() method
        /// to abort the transmission.
        /// </remarks>
        virtual bool begin_send() = 0;

        /// <summary>
        /// Begin transmission of the buffer to the given address.
        /// </summary>
        /// <remarks>
        /// The poll() method must also be invoked with the rules listed in the
        /// remarks after calling this method.
        /// </remarks>
        virtual void send() = 0;

        /// <summary>
        /// Aborts any pending response and returns the state machine to the
        /// idle state.
        /// </summary>
        /// <remarks>
        /// The poll() method must also be invoked with the rules listed in the
        /// remarks after calling this method.
        /// </remarks>
        virtual void finished() = 0;

        /// <summary>
        /// Indicates that a frame has been received for our station address or broadcast.
        /// </summary>
        /// <remarks>
        /// If this returns true, the buffer must be released using the
        /// finished() method, or starting a new transmission using
        /// begin_send() and following the respective process.
        /// </remarks>
        virtual bool frame_ready() const = 0;

        /// <summary>
        /// Returns the station address for the PDU, or 0 if broadcast or point-to-point.
        /// </summary>
        virtual uint8_t frame_address() const = 0;

        /// <summary>
        /// Sets the station address for the PDU, or 0 if broadcast or point-to-point.
        /// </summary>
        virtual void set_frame_address(uint8_t address) = 0;

        /// <summary>
        /// Gets the PDU buffer pointer.
        /// </summary>
        /// <remarks>
        /// Regardless of the protocol used, the first byte will always be the
        /// function code, and the bytes that follow are the data bytes.
        ///
        /// The station address and checksum bytes are not included in this
        /// buffer.
        /// </remarks>
        virtual uint8_t* buffer() = 0;

        /// <summary>
        /// Returns the number of data bytes in the PDU buffer.
        /// </summary>
        /// <remarks>
        /// This value includes the function code byte, and data bytes, but
        /// excludes the station address and checksum bytes.
        /// </remarks>
        virtual size_t buffer_len() const = 0;

        /// <summary>
        /// Sets the length of the buffer.
        /// </summary>
        /// <remarks>
        /// This value includes the function code byte, and data bytes, but
        /// excludes the station address and checksum bytes.
        /// </remarks>
        virtual void set_buffer_len(size_t len) = 0;

        /// <summary>
        /// Returns the maximum allowable length of the buffer.
        /// </summary>
        virtual size_t buffer_max() const = 0;
    };

    /// <summary>
    /// Represents the slave protocol handling.
    /// </summary>
    class ISlave
    {
    public:
        virtual ~ISlave() {}

        /// <summary>
        /// Executes the frame ready callback.
        /// </summary>
        virtual void frame_ready() = 0;

        /// <summary>
        /// Helper to convert the C style callback on IFramer to an instance of this class.
        /// </summary>
        /// <remarks>
        /// This helper can be used as follows:
        /// <pre>
        /// framer->set_frame_ready_callback(&ISlave::frame_ready_callback, &slave);
        /// </pre>
        /// </remarks>
        static void frame_ready_callback(void* obj) { ((ISlave*)obj)->frame_ready(); }
    };
}
#endif
