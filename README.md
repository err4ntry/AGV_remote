基于NUCLEO-F446RE开发板的AGV主控代码
遥控器的控制方式如下：

![image text](https://github.com/err4ntry/AGV_remote/blob/master/picture/%E5%9B%BE%E7%89%871.png "DBSCAN Performance Comparison")

CH1：左摇杆上下移动
CH2：右摇杆左右移动
CH3：右摇杆上下移动
CH4：左摇杆左右移动
CH5：SA
CH6：SB
CH7：SF
CH8：SC
CH9：SD
CH10：SH
CH11：S1
CH12：S2

控制方式：
1.SA三档使能，一档失能
2.CH1（左摇杆上下）调速度
3.SF切换前后
4.CH2（右摇杆左右）控制转弯
5.SD二档或三档控制继电器（急停）

//位置模式：（必须先使能再切换位置模式）
1.SF一档为直行模式，二挡为转弯模式
2.S1调位置
3.位置模式下右摇杆向右触发
