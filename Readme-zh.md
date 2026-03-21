# ESP32C3 蓝牙键鼠转 PS2 适配器

基于 ESP32C3 + ESP‑IDF V5.5.0 开发的蓝牙键盘 / 鼠标转 PS2 协议转换设备，
支持双主机、四路 PS2 输出、I2C LCD1602 状态显示、蓝牙配对与电量监控。

## 功能概述
- 蓝牙键盘、蓝牙鼠标 → 双路 PS2 键盘 + 双路 PS2 鼠标
- 同时支持两台 PC 的 PS2 接口
- 四路 PS2 共用一个硬件定时器，资源高效
- LCD1602 实时显示选择状态、初始化、锁键、蓝牙连接、电量、配对码、异常指令

## 硬件引脚定义

### PS2 接口
| 功能 | GPIO |
|------|------|
| Key1 CLK | 19 |
| Key1 DAT | 13 |
| Mouse1 CLK | 12 |
| Mouse1 DAT | 18 |
| Key2 CLK | 2 |
| Key2 DAT | 3 |
| Mouse2 CLK | 10 |
| Mouse2 DAT | 6 |

### LCD1602（I2C）
| 功能 | GPIO |
|------|------|
| SCL | 5 |
| SDA | 4 |

## LCD1602 显示规则
第一行：PC1 状态  
第二行：PC2 状态

位置含义：
1：`>` = 当前选中主机  
2：S=键盘就绪，s=未就绪/异常  
3：1=NumLock 开，←=关  
4：A=CapsLock 开，a=关  
5：-=ScrollLock 开，|=关  
6：K=键盘蓝牙已连 / k=未连  
7：M=鼠标蓝牙已连 / m=未连  
8~13：蓝牙配对码（优先级低于异常指令）  
第12位：K/M + 4位十六进制 = 未识别主机指令  
第二行6~8：键盘电量  (未正常工作)
第二行9~11：鼠标电量 （未正常工作）

## 工程依赖
- ESP-IDF：v5.5.0
- 驱动来源：
  - Reference/ps2：四路 PS2 驱动（共用定时器，已验证）
  - Reference/lcd1602：I2C 显示屏驱动（已验证）

## 编译 & 烧录
```bash
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
