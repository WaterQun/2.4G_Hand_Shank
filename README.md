# 2.4G Hand Shank
#### 基于STM32F103的2.4G无线控制手柄，发送2.4G无线信号，控制其它设备端

---

## 一、项目说明/Overview
- 关于**硬件**，Hardware文件夹内是2.4G手柄里面用到的所有电路的原理图和PCB文件，目前提供的是Altium Designer软件格式的源文件用于后续自己修改以及提供给厂家进行加工。
- 关于**软件**，Software中提供的是2.4G手柄的MDK代码，可以使用keil软件打开项目工程进行仿真烧录，或者直接使用FlyMCU对OBJ输出文件夹中的Hex文件进行烧录。
- 另外，在Documents中含有相关的**参考文件**，包括芯片的Datasheet等，用于后续修改参考。

## 二、硬件说明/Hardware
本项目硬件基于[技小新团队/2.4G手柄遥控器 (oshwhub.com)](https://oshwhub.com/jixin/2_4G-56cf3a971e094f78885dc230ecbe10d2)有部分更改，2.4G模块通信流程已通过实物测试并在比赛中使用，暂未发现问题。
板子还是大一的时候画的（能用就行），后面发现了一些可以改进的地方，比如电源管理芯片不常见等问题，我有空会打算再重新画块板子，以及针对串口透传进行一些测试。

## 三、软件说明/SoftWare
目前根据需要，是适配的四轴飞行器编写的控制按键，可根据需要自行编写。

> 感谢以下项目：
>
> [技小新团队/2.4G手柄遥控器 (oshwhub.com)](https://oshwhub.com/jixin/2_4G-56cf3a971e094f78885dc230ecbe10d2)
>
> 以及电子科技协会的前辈 **@假人** 等学长代码的借鉴及谆谆教导！

