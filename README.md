# dynamixelPlus-Chinese ROS
中文注释的 Dynamixel 伺服和传感器驱动 ，方便国内用户使用。可直接集成入ROS，在KINETIC版本中测试通过。

## 对Robotis驱动的修改

Dynamixel 的驱动代码来自 Robotis， 我们进行了少量的优化，修正了一些BUG。

改动较大的地方是两处：

* Sync write 的指令没有响应数据包，为了提高发送的可靠性，我们为每一个指令数据包添加了一个8位的序列号，以便在后续的响应包发现传输失败，进行重发。
* Read 指令可以根据实际读到的数据进行调整，而不是固定为请求读取的字节数



## 增加的伺服功能
天乐思的伺服，增加了一些方便进行位置，速度和转矩控制的控制模式。具体请在天乐思网站上查找对应伺服的数据手册

## 增加的传感器
天乐思提供了多达十种传感器模块，方便快速搭建机器人原型，各型号传感器的功能和寄存器描述，请在天乐思网站上查找对应传感器的数据手册

