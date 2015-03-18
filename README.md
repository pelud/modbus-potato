# modbus-potato
C++ Embedded Modbus Library

Features:
 * object oriented C++
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