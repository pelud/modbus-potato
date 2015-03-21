#include "stdafx.h"
#include "../../../../ModbusRTU.h"
#include "../../../../ModbusASCII.h"
#include <stdexcept>
#include <vector>
#include <tuple>
#include <string>
#undef min

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace Microsoft::VisualStudio::TestTools::UnitTesting;
using namespace ModbusPotato;

namespace UnitTests
{
#pragma region Dummy Classes
    class CDummyStream : public ModbusPotato::IStream, public ModbusPotato::ITimeProvider
    {
    public:
        CDummyStream(
            std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > const& items = std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> >())
            :   m_pos()
            ,   m_col()
            ,   m_items(items)
            ,   m_time()
            ,   m_last_write()
            ,   m_rx_status()
            ,   m_tx_status()
            ,   m_rx_on_count()
            ,   m_tx_on_count()
        {
        }
        virtual int read(uint8_t* buffer, size_t buffer_size)
        {
            if (m_pos >= m_items.size())
                return 0;

            auto& start = std::get<0>(m_items[m_pos]);
            auto& data = std::get<1>(m_items[m_pos]);

            if (ticks() >= start)
            {
                auto len = std::min(data.size() - m_col, buffer_size);
                if (buffer)
                    std::copy(data.begin() + m_col, data.begin() + m_col + len, buffer);
                m_col += len;
                if (m_col == data.size())
                {
                    m_pos++;
                    m_col = 0;
                }
                return len;
            }
            return 0;
        }
        virtual int write(uint8_t* buffer, size_t len)
        {
            if (!len || ticks() == m_last_write)
                return 0;
            m_write.push_back(std::tr1::make_tuple(m_time, std::string(buffer, buffer + len)));
            m_last_write = ticks();
            write_data.insert(write_data.end(), buffer, buffer + len);
            return len;
        }
        virtual void txEnable(bool state)
        {
        }
        virtual bool writeComplete()
        {
            return true;
        }
        virtual system_tick_t ticks() const
        {
            return m_time;
        }
        virtual unsigned long microseconds_per_tick() const { return 1000; }
        virtual void communicationStatus(bool rx, bool tx)
        {
            if (rx)
                m_rx_on_count++;
            if (tx)
                m_tx_on_count++;
            m_rx_status = rx;
            m_tx_status = tx;
        }
        void increment(system_tick_t value)
        {
            m_time += value;
        }
        void written(std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> >& result) const { result = m_write; }
        std::string write_data;
        bool m_rx_status, m_tx_status;
        int m_rx_on_count, m_tx_on_count;
    private:
        system_tick_t m_time, m_last_write;
        size_t m_pos, m_col;
        std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > m_items, m_write;
    };
#pragma endregion

    [TestClass]
    public ref class RTUTests
    {
    public: 

        [TestMethod]
        void TestRTUConstructor()
        {
            CDummyStream stream;
            uint8_t buffer[MODBUS_DATA_BUFFER_SIZE];
            CModbusRTU rtu(&stream, &stream, buffer, _countof(buffer));
            rtu.setup(9600);
            Assert::AreEqual(true, true);
            Assert::AreEqual(false, stream.m_rx_status);
            Assert::AreEqual(false, stream.m_tx_status);
            Assert::AreEqual(0, stream.m_rx_on_count);
            Assert::AreEqual(0, stream.m_tx_on_count);
        };

        [TestMethod]
        void TestReceiveRTUFrame()
        {
            std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > items;

            // provide some junk to dump at 1.5ms
            uint8_t frame1[] = { 2, 7, 0x41, 0x12 };
            items.push_back(std::tr1::make_tuple(1, std::string(frame1, frame1 + _countof(frame1))));

            // incoming datagram at 5.51ms
            items.push_back(std::tr1::make_tuple(5, std::string(frame1, frame1 + _countof(frame1))));

            // parse the frames
            CDummyStream stream(items);
            uint8_t buffer[MODBUS_DATA_BUFFER_SIZE];
            CModbusRTU rtu(&stream, &stream, buffer, _countof(buffer));
            rtu.setup(9600);

            while (stream.ticks() < 10)
            {
                rtu.poll();
                stream.increment(1);
            }

            // check the result
            Assert::AreEqual(true, rtu.frame_ready());
            Assert::AreEqual((byte)2, rtu.frame_address());
            Assert::AreEqual(1u, rtu.buffer_len());
            Assert::AreEqual((byte)7, rtu.buffer()[0]);
            Assert::AreEqual(false, stream.m_rx_status);
            Assert::AreEqual(false, stream.m_tx_status);
            Assert::AreEqual(1, stream.m_rx_on_count); // the first frame is not counted because it's still starting up
            Assert::AreEqual(0, stream.m_tx_on_count);
        };

        [TestMethod]
        void TestReceiveASCIIFrame()
        {
            std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > items;

            // incoming datagram at 5.51ms
            uint8_t frame1[] = ":1103006B00037E\r\n";
            items.push_back(std::tr1::make_tuple(5, std::string(frame1, frame1 + _countof(frame1) - 1)));

            // parse the frames
            CDummyStream stream(items);
            uint8_t buffer[MODBUS_DATA_BUFFER_SIZE];
            CModbusASCII framer(&stream, &stream, buffer, _countof(buffer));

            while (stream.ticks() < 10)
            {
                framer.poll();
                stream.increment(1);
            }

            // check the result
            Assert::AreEqual(true, framer.frame_ready());
            Assert::AreEqual((byte)17, framer.frame_address());
            uint8_t response[] = { 0x03, 0x00, 0x6B, 0x00, 0x03 };
            Assert::AreEqual((size_t)_countof(response), framer.buffer_len());
            Assert::AreEqual(true, std::equal(response, response + _countof(response), framer.buffer()));
            Assert::AreEqual(false, stream.m_rx_status);
            Assert::AreEqual(false, stream.m_tx_status);
            Assert::AreEqual(1, stream.m_rx_on_count);
            Assert::AreEqual(0, stream.m_tx_on_count);
        };

        [TestMethod]
        void TestReceiveInputOverflow()
        {
            std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > items;

            // provide some junk to dump at 4ms
            items.push_back(std::tr1::make_tuple(4, std::string(0x200, 'a')));

            // create the rtu object
            CDummyStream stream(items);
            uint8_t buffer[MODBUS_DATA_BUFFER_SIZE];
            CModbusRTU rtu(&stream, &stream, buffer, _countof(buffer));
            rtu.setup(9600);

            // parse the frames
            while (stream.ticks() < 10)
            {
                rtu.poll();
                stream.increment(1);
            }

            // check the result
            Assert::AreEqual(false, rtu.frame_ready());
            Assert::AreEqual(false, stream.m_rx_status);
            Assert::AreEqual(false, stream.m_tx_status);
        };

        [TestMethod]
        void TestRTUTransmitFrame()
        {
            CDummyStream stream;
            uint8_t buffer[MODBUS_DATA_BUFFER_SIZE];
            CModbusRTU rtu(&stream, &stream, buffer, _countof(buffer));
            rtu.setup(9600);

            // skip some ticks to wait for the initial dump
            while (stream.ticks() < 5)
            {
                rtu.poll();
                stream.increment(1);
            }

            // aquire the buffer
            Assert::AreEqual(true, rtu.begin_send());

            // update the data
            rtu.set_frame_address(2);
            rtu.buffer()[0] = 7;
            rtu.set_buffer_len(1);

            // begin the send
            rtu.send();

            // wait for the transfer to happen
            while (stream.ticks() < 10)
            {
                rtu.poll();
                stream.increment(1);
            }

            // get the result
            std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > items;
            stream.written(items);


            Assert::AreEqual(false, stream.m_rx_status);
            Assert::AreEqual(false, stream.m_tx_status);
            Assert::AreEqual(0, stream.m_rx_on_count);
            Assert::AreEqual(1, stream.m_tx_on_count);
        }

        [TestMethod]
        void TestASCIITransmitFrame()
        {
            CDummyStream stream;
            uint8_t buffer[MODBUS_DATA_BUFFER_SIZE];
            CModbusASCII rtu(&stream, &stream, buffer, _countof(buffer));

            // skip some ticks to wait for the initial dump
            while (stream.ticks() < 5)
            {
                rtu.poll();
                stream.increment(1);
            }

            // aquire the buffer
            Assert::AreEqual(true, rtu.begin_send());

            // update the data
            rtu.set_frame_address(17);
            uint8_t data[] = { 0x03, 0x00, 0x6B, 0x00, 0x03 };
            std::copy(data, data + _countof(data), rtu.buffer());
            rtu.set_buffer_len(_countof(data));

            // begin the send
            rtu.send();

            // wait for the transfer to happen
            while (stream.ticks() < 30)
            {
                rtu.poll();
                stream.increment(1);
            }

            // check the result
            Assert::AreEqual(gcnew System::String(":1103006B00037E\r\n"), gcnew System::String(stream.write_data.c_str()));
            Assert::AreEqual(false, stream.m_rx_status);
            Assert::AreEqual(false, stream.m_tx_status);
            Assert::AreEqual(0, stream.m_rx_on_count);
            Assert::AreEqual(1, stream.m_tx_on_count);
        }
    };
}
