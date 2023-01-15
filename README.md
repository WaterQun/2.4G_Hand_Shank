# 2.4G Hand Shank
#### 基于STM32F103的2.4G无线控制手柄，发送2.4G无线信号，控制其它设备端

---

## 一、项目文件说明/Overview
### 1、Hardware
Hardware文件夹内是2.4G手柄里面用到的所有电路的原理图和PCB文件，目前提供的是Altium Designer软件格式的源文件用于后续自己修改以及提供给厂家进行加工。

### 2、Software
Software中提供的是2.4G手柄的MDK代码，可以使用keil软件打开项目工程进行仿真烧录，或者直接使用FlyMCU对OBJ输出文件夹中的Hex文件进行烧录。

### 3、Documents
相关的参考文件，包括芯片的Datasheet等，用于后续修改参考。

## 二、硬件说明/Hardware
工作电压：3.3V～6V
供电方式：3.7V 锂电池
工作频段：2.4GHz ISM 频段
接口：OLED 显示屏、串口透传模块、SWD 下载口

**关于烧录方式？**
使用JLink、STLink之类的调试器烧录，在PCB上预留了SWD调试口。

## 三、软件说明/SoftWare

**关于键盘固件的按键映射方式？**
目前根据需要，是适配的四轴飞行器编写的控制按键，可根据需要自行编写。

> 感谢以下项目：
>
> [技小新团队/2.4G手柄遥控器 (oshwhub.com)](https://oshwhub.com/jixin/2_4G-56cf3a971e094f78885dc230ecbe10d2)
>
> 以及湖南工学院电子科技协会的前辈 **@假人** 等学长代码的借鉴及谆谆教导！

