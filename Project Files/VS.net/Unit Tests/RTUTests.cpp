#include "stdafx.h"
#include "../../../RTU.h"
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
        void increment(system_tick_t value)
        {
            m_time += value;
        }
        void written(std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> >& result) const { result = m_write; }
    private:
        system_tick_t m_time, m_last_write;
        size_t m_pos, m_col;
        std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > m_items, m_write;
    };
#pragma endregion

    [TestClass]
    public ref class RTUTests
    {
    private:
        TestContext^ testContextInstance;

    public: 
        /// <summary>
        ///Gets or sets the test context which provides
        ///information about and functionality for the current test run.
        ///</summary>
        property Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ TestContext
        {
            Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ get()
            {
                return testContextInstance;
            }
            System::Void set(Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ value)
            {
                testContextInstance = value;
            }
        };

        [TestMethod]
        void TestRTUConstructor()
        {
            CDummyStream stream;
            CRTU rtu(&stream, &stream, 9600);
            Assert::AreEqual(true, true);
        };

        [TestMethod]
        void TestReceiveFrame()
        {
            std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > items;

            // provide some junk to dump at 1.5ms
            uint8_t frame1[] = { 2, 7, 0x41, 0x12 };
            items.push_back(std::tr1::make_tuple(1, std::string(frame1, frame1 + _countof(frame1))));

            // incoming datagram at 5.51ms
            items.push_back(std::tr1::make_tuple(5, std::string(frame1, frame1 + _countof(frame1))));

            // parse the frames
            CDummyStream stream(items);
            CRTU rtu(&stream, &stream, 9600);
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
        };

        [TestMethod]
        void TestReceiveInputOverflow()
        {
            std::vector<std::tr1::tuple<system_tick_t /*start*/, std::string /*data*/> > items;

            // provide some junk to dump at 4ms
            items.push_back(std::tr1::make_tuple(4, std::string(0x200, 'a')));

            // parse the frames
            CDummyStream stream(items);
            CRTU rtu(&stream, &stream, 9600);
            while (stream.ticks() < 10)
            {
                rtu.poll();
                stream.increment(1);
            }

            // check the result
            Assert::AreEqual(false, rtu.frame_ready());
        };

        [TestMethod]
        void TestTransmitFrame()
        {
            CDummyStream stream;
            CRTU rtu(&stream, &stream, 9600);

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
        }
    };
}
