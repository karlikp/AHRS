# 🧭 Attitude and Heading Reference System for a smart valet
I take part in project "Developing a smart valet parking system that utilizes optimal space". where I'm responsible for integrate  

## Table of contents:
1. [Introduction](#1-introduction)
2. [Requirements](#2-requirements)
3. [Installation](#3-installation)
4. [Usage](#4-usage)
5. [Gallery](#5-gallery)
   
## 1. Introduction:

The program is a module that enables data acquisition from distance sensors, IMU, and the Unitree LM1 RM LiDAR. The collected data is then processed to create a transformation matrix, which allows for estimating changes in position and orientation of the vehicle.
Subsequently, the complete dataset is transmitted to the appropriate nodes via the MQTT protocol to a database.



## 2. Requirements:
  - 2.1. Operating system:
       - Linux (tested on Ubuntu 22.04 LTS)
        
  - 2.2. Dependence:
       - python3
       - adafruit-circuitpython-bmp3xx
       - adafruit-circuitpython-vl53l1x
       - adafruit-circuitpython-vl6180x
       - smbus2

## 3. Installation:
 - 3.1. Dependence for Linux (ubuntu 22.04):
```bash
sudo apt update
sudo apt install python3
```
Creating python virtual environment(optional):
```bash
python3 -m venv your_env
```

activate the virtual environent(optional):
```bash
source your_env/bin/activate
```
Needed libraries:
```bash
pip install adafruit-circuitpython-bmp3xx
pip install smbus2
pip install adafruit-circuitpython-vl53l1x
pip install adafruit-circuitpython-vl6180x
```
## 4. Usage:
```bash
cd path/to/AHRS
python3 main.py
```

## 5. Gallery:

  <!-- TO DO
adding screen from runtime-->

<div align="center">
  <img src="https://github.com/user-attachments/assets/11971815-8101-4355-8803-93c04207fe87" alt="menu">
  <p>Fig. 1. Use case diagram</p>
</div>
<br>



