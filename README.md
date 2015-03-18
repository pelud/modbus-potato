# modbus-potato
C++ Embedded Modbus Library

Features:
 * object oriented C++
 * currently supports modbus RTU slave with read holding registers and write multiple registers
 * most methods and functions are unit tested
 * easy to use
 * liberal license (MIT)
 * non-blocking state machine based RTU framer design
```
                     -------------          -------------
         +--Sent-->|   TX CRC    |--Sent-->|   TX Wait   |--done--+    start
         |           -------------          -------------         |      |
   -------------                                                  v      v
  |   TX PDU    |                                               -------------
   -------------                              +----------------|    Dump     |
         ^                                    |                 -------------
         |                                  T3.5                      |
       Sent         +----begin_send()---+     |                       |
         |          |                   |     v                       |
   -------------    |                -------------                    |
  |   TX Addr   |   |     +-------->|    Idle     |---Invalid Char--->+
   -------------    |     |          -------------                    ^
       ^            |     |                |                          |
       |     +------+     |          Address Match                    |
     send()  |            |                |                          |
       |     v            |                v                          |
   -------------  fini-   |   T3.5+  -------------                    |
+-|    Queue    |-shed()->+<--CRC/--|   Receive   |---T1.5/Comm Err-->+
|  -------------          ^   F.E.   -------------                    ^
|        ^                |                |                          |
|        |                |          T3.5+Frame OK            finished()/send()
|   begin_send()      finished()           |                          |
|        |                |                v                          |
|        |                |          -------------              ------------- 
|        +----------------+---------| Frame Ready |--Receive-->|  Collision  |
|                                    -------------              -------------
|                                                                     ^
|                                                                     |
+----Receive----------------------------------------------------------+
```

For class descriptions see the src/ModbusInterface.h file.

This project follows the Arduino library format version 2, so you can select
"Download ZIP" from the github page and then select the "Add Library..." option
from the Arduino environment to easily import this library into your
environment.

The "modpoll" command line utility is highly recommended for accessing slaves.
It can be found here: http://www.modbusdriver.com/modpoll.html

To install it, copy the executable to some location in your path, such as
C:\Windows (for Windows) or /usr/local/bin (for Linux).

You can read the a holding register (4xxxxx) with modpoll using the
following command line:
`modpoll -m rtu -a <slave address> -t 4 -r <address> -b <baud> <COMx:>`

You can write to a register using the following:
`modpoll -m rtu -a <slave address> -t 4 -r <address> -b <baud> <COMx:> <value>`

Replace `<slave address>` with the address of your slave (i.e. 1), `<register>` 
with the 1 based register number (i.e. 1=40001), `<baud>` with the baud rate
(i.e. 19200), `<COMx:>` with your COM port (i.e. COM4: or /dev/ttyS1) and
`<value>` with the value to write.  You may omit `-m rtu` on if the device
name starts with COM, and you may omit `-b <baud>` if the baud rate is
19200.

For example, to read the first holding register (40001) from slave id #1 on
COM3: at 19200 baud, you can use the following:
`modpoll -1 -a 1 -t 4 -r 1 COM3:`

To write the value 100 to the above register, use the following:
`modpoll -a 1 -t 4 -r 1 COM3: 100`

Please be aware that modpoll will assert the DTR line when it opens the port,
which will usually cause an Arduino to reset.  You may need to disable the DTR
reset support on the board, or use a serial to TCP bridge program like piracom
(http://www.pira.cz/show.asp?art=piracom) which keeps the port open and connect
to the TCP socket instead.  If you use a serial to tcp bridge, change the
`-m rtu` command line switch to `-m enc`.

To use modpoll to read the above register through piracom running on the local
machine, use:
`modpoll -1 -m enc -a 1 -t 4 -r 1 localhost`

