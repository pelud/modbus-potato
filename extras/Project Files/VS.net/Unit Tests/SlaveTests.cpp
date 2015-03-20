#include "stdafx.h"
#include "../../../../ModbusSlave.h"
#include "../../../../ModbusSlaveHandlerBase.h"
#include <algorithm>
#pragma comment(lib, "Ws2_32.lib")

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;
using namespace ModbusPotato;

namespace UnitTests
{
#pragma region Dummy Classes
    public class CFramerDummy : public IFramer
    {
    public:
        CFramerDummy()
            :   m_station_address()
            ,   m_frame_address()
            ,   m_buffer_len()
            ,   was_sent()
            ,   was_finished()
        {
        }
        virtual void set_handler(IFrameHandler* handler) { }
        virtual uint8_t station_address() const { return m_station_address; }
        virtual void set_station_address(uint8_t address) { m_station_address = address; }
        virtual unsigned long poll() { return 0; }
        virtual bool begin_send() { return true; }
        virtual void send() { was_sent = true; }
        virtual void finished() { was_finished = true; }
        virtual bool frame_ready() const { return true; }
        virtual uint8_t frame_address() const { return m_frame_address; }
        virtual void set_frame_address(uint8_t address) { m_frame_address = address; }
        virtual uint8_t* buffer() { return m_buffer; }
        virtual size_t buffer_len() const { return m_buffer_len; }
        virtual void set_buffer_len(size_t len) { m_buffer_len = len; }
        virtual size_t buffer_max() const { return _countof(m_buffer); }
        bool was_sent;
        bool was_finished;
    private:
        uint8_t m_station_address, m_frame_address;
        size_t m_buffer_len;
        uint8_t m_buffer[256];
    };

    class CSlaveHandler : public CModbusSlaveHandlerBase
    {
    public:
        CSlaveHandler()
            :   last_address()
            ,   last_count()
        {
            std::fill(last_values, last_values + _countof(last_values), 0);
        }
        uint16_t last_address, last_count;
        uint16_t last_values[256];
        virtual modbus_exception_code::modbus_exception_code read_coils(uint16_t address, uint16_t count, uint8_t* result)
        {
            last_address = address;
            last_count = count;
            static const uint8_t data[] = { 0xCD, 0x6B, 0xB2, 0x0E, 0x1B };
            for (uint16_t i = 0; i < count; ++i)
                result[i] = data[i % _countof(data)];
            return modbus_exception_code::ok;
        }
        virtual modbus_exception_code::modbus_exception_code read_discrete_inputs(uint16_t address, uint16_t count, uint8_t* result)
        {
            last_address = address;
            last_count = count;
            static const uint8_t data[] = { 0xAC, 0xDB, 0x35 };
            for (uint16_t i = 0; i < count; ++i)
                result[i] = data[i % _countof(data)];
            return modbus_exception_code::ok;
        }
        virtual modbus_exception_code::modbus_exception_code read_holding_registers(uint16_t address, uint16_t count, uint16_t* result)
        {
            last_address = address;
            last_count = count;
            static const uint16_t data[] = { 0xAE41, 0x5652, 0x4340 };
            for (uint16_t i = 0; i < count; ++i)
                result[i] = data[i % _countof(data)];
            return modbus_exception_code::ok;
        }
        virtual modbus_exception_code::modbus_exception_code read_input_registers(uint16_t address, uint16_t count, uint16_t* result)
        {
            last_address = address;
            last_count = count;
            static const uint16_t data[] = { 0x000A, 0x5652, 0x4340 };
            for (uint16_t i = 0; i < count; ++i)
                result[i] = data[i % _countof(data)];
            return modbus_exception_code::ok;
        }
        virtual modbus_exception_code::modbus_exception_code write_multiple_registers(uint16_t address, uint16_t count, const uint16_t* values)
        {
            last_address = address;
            last_count = count;
            std::copy(values, values + count, last_values);
            return modbus_exception_code::ok;
        }
        virtual modbus_exception_code::modbus_exception_code write_multiple_coils(uint16_t address, uint16_t count, const uint8_t* values)
        {
            last_address = address;
            last_count = count;
            std::copy(values, values + count, last_values);
            return modbus_exception_code::ok;
        }
    };

#pragma endregion

	[TestClass]
	public ref class SlaveTests
	{
	public: 
        [TestMethod]
		void TestSlaveConstructor()
		{
            CModbusSlave slave(NULL);
		}

        [TestMethod]
		void TestSlaveException()
		{
            // create the slave object
            CModbusSlave slave(NULL);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC03.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x03, 0x00, 0x6B, 0x00, 0x03 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint8_t)0x83, framer.buffer()[0]); // read holding registers exception
            Assert::AreEqual((uint8_t)0x01, framer.buffer()[1]); // illegal function
        }

        // FC01
        [TestMethod]
		void TestSlaveFC01ReadCoilStatus()
		{
            // create the slave object
            CSlaveHandler handler;
            CModbusSlave slave(&handler);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC01.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x01, 0x00, 0x13, 0x00, 0x25 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreNotEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint16_t)0x0013, handler.last_address);
            Assert::AreEqual((uint16_t)0x0025, handler.last_count);

            // data packet
            uint8_t response[] = { 0x01, 0x05, 0xCD, 0x6B, 0xB2, 0x0E, 0x1B };
            Assert::AreEqual((size_t)_countof(response), framer.buffer_len());
            Assert::AreEqual(true, std::equal(response, response + _countof(response), framer.buffer()));
        }

        // FC02
        [TestMethod]
		void TestSlaveFC02ReadDiscreteInputStatus()
		{
            // create the slave object
            CSlaveHandler handler;
            CModbusSlave slave(&handler);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC02.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x02, 0x00, 0xC4, 0x00, 0x16 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreNotEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint16_t)0x00C4, handler.last_address);
            Assert::AreEqual((uint16_t)0x0016, handler.last_count);

            // data packet
            uint8_t response[] = { 0x02, 0x03, 0xAC, 0xDB, 0x35 };
            Assert::AreEqual((size_t)_countof(response), framer.buffer_len());
            Assert::AreEqual(true, std::equal(response, response + _countof(response), framer.buffer()));
        }

        // FC03
        [TestMethod]
		void TestSlaveFC03ReadHoldingRegister()
		{
            // create the slave object
            CSlaveHandler handler;
            CModbusSlave slave(&handler);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC03.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x03, 0x00, 0x6B, 0x00, 0x03 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreNotEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint16_t)0x006b, handler.last_address);
            Assert::AreEqual((uint16_t)0x0003, handler.last_count);
            Assert::AreEqual((size_t)8, framer.buffer_len());
            Assert::AreEqual((uint8_t)0x03, framer.buffer()[0]); // read holding registers
            Assert::AreEqual((uint8_t)0x06, framer.buffer()[1]); // number of bytes to follow
            Assert::AreEqual((uint8_t)0xAE, framer.buffer()[2]); // data 0 H
            Assert::AreEqual((uint8_t)0x41, framer.buffer()[3]); // data 0 L
            Assert::AreEqual((uint8_t)0x56, framer.buffer()[4]); // data 1 H
            Assert::AreEqual((uint8_t)0x52, framer.buffer()[5]); // data 1 L
            Assert::AreEqual((uint8_t)0x43, framer.buffer()[6]); // data 2 H
            Assert::AreEqual((uint8_t)0x40, framer.buffer()[7]); // data 2 L
		}

        // FC04
        [TestMethod]
		void TestSlaveFC04ReadInputRegisterStatus()
		{
            // create the slave object
            CSlaveHandler handler;
            CModbusSlave slave(&handler);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC04.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x04, 0x00, 0x08, 0x00, 0x01 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreNotEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint16_t)0x0008, handler.last_address);
            Assert::AreEqual((uint16_t)0x0001, handler.last_count);

            // data packet
            uint8_t response[] = { 0x04, 0x02, 0x00, 0x0A };
            Assert::AreEqual((size_t)_countof(response), framer.buffer_len());
            Assert::AreEqual(true, std::equal(response, response + _countof(response), framer.buffer()));
        }

        // FC05
        [TestMethod]
		void TestSlaveFC05ForceSingleCoil()
		{
            // create the slave object
            CSlaveHandler handler;
            CModbusSlave slave(&handler);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC05.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x05, 0x00, 0xAC, 0xFF, 0x00 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreNotEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint16_t)0x00ac, handler.last_address);
            Assert::AreEqual((uint16_t)0x0001, handler.last_count);
            Assert::AreEqual((uint16_t)0x01, handler.last_values[0]);

            // data packet
            Assert::AreEqual((size_t)_countof(data), framer.buffer_len());
            Assert::AreEqual(true, std::equal(data, data + _countof(data), framer.buffer()));
        }

        // FC06
        [TestMethod]
		void TestSlaveFC06PresetSingleRegister()
		{
            // create the slave object
            CSlaveHandler handler;
            CModbusSlave slave(&handler);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC06.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x06, 0x00, 0x01, 0x00, 0x03 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreNotEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint16_t)0x0003, handler.last_values[0]);
            Assert::AreEqual((uint16_t)0x0001, handler.last_address);
            Assert::AreEqual((uint16_t)0x0001, handler.last_count);
            Assert::AreEqual((size_t)5, framer.buffer_len());
            Assert::AreEqual((uint8_t)0x06, framer.buffer()[0]); // preset single register
            Assert::AreEqual((uint8_t)0x00, framer.buffer()[1]); // address H
            Assert::AreEqual((uint8_t)0x01, framer.buffer()[2]); // address L
            Assert::AreEqual((uint8_t)0x00, framer.buffer()[3]); // value H
            Assert::AreEqual((uint8_t)0x03, framer.buffer()[4]); // value L
        }

        // FC15
        [TestMethod]
		void TestSlaveFC15ForceMultipleCoils()
		{
            // create the slave object
            CSlaveHandler handler;
            CModbusSlave slave(&handler);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC15.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x0F, 0x00, 0x13, 0x00, 0x0A, 0x02, 0xCD, 0x01 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreNotEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint16_t)0x0013, handler.last_address);
            Assert::AreEqual((uint16_t)0x000A, handler.last_count);
            Assert::AreEqual((uint16_t)0xCD, handler.last_values[0]);
            Assert::AreEqual((uint16_t)0x01, handler.last_values[1]);

            // data packet
            uint8_t response[] = { 0x0F, 0x00, 0x13, 0x00, 0x0A };
            Assert::AreEqual((size_t)_countof(response), framer.buffer_len());
            Assert::AreEqual(true, std::equal(response, response + _countof(response), framer.buffer()));
        }

        // FC16
        [TestMethod]
		void TestSlaveFC16WriteMultipleRegisters()
		{
            // create the slave object
            CSlaveHandler handler;
            CModbusSlave slave(&handler);

            // initialize with a test packet
            // from http://www.simplymodbus.ca/FC16.htm
            CFramerDummy framer;
            framer.set_frame_address(0x11);
            uint8_t data[] = { 0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02 };
            std::copy(data, data + _countof(data), framer.buffer());
            framer.set_buffer_len(_countof(data));

            // simulate the frame received event
            slave.frame_ready(&framer);

            // check the result
            Assert::AreEqual(true, framer.was_sent);
            Assert::AreNotEqual((size_t)2, framer.buffer_len());
            Assert::AreEqual((uint16_t)0x000a, handler.last_values[0]);
            Assert::AreEqual((uint16_t)0x0102, handler.last_values[1]);
            Assert::AreEqual((uint16_t)0x0001, handler.last_address);
            Assert::AreEqual((uint16_t)0x0002, handler.last_count);
            Assert::AreEqual((size_t)5, framer.buffer_len());
            Assert::AreEqual((uint8_t)0x10, framer.buffer()[0]); // write multiple
            Assert::AreEqual((uint8_t)0x00, framer.buffer()[1]); // address H
            Assert::AreEqual((uint8_t)0x01, framer.buffer()[2]); // address L
            Assert::AreEqual((uint8_t)0x00, framer.buffer()[3]); // count H
            Assert::AreEqual((uint8_t)0x02, framer.buffer()[4]); // count L
		}
    };
}
