# Simultaneous localization and mapping for a smart valet
I take part in project "Developing a smart valet parking system that utilizes optimal space". where I'm responsible for integrate  

## Description
The project utilizes the A* algorithm to optimize the placement of cars in the parking area.
The team is tasked with developing a prototype of a valet system capable of repositioning vehicles dynamically.

My responsibility is to integrate sensors to create complex system which let appoint valet position and creating vehicle surroundings.

### Requirements
The python program was tested on "Debian GNU/Linux 12 (bookworm).

I advise use python3.
If you don't have it, you can install it by:
```bash
sudo apt update
sudo apt install python3
```

Installed libraries:
- adafruit-circuitpython-bmp3xx,
- adafruit-circuitpython-vl53l1x,
- adafruit-circuitpython-vl6180x,
- smbus2.

If you don't have the libraries, you should install them:
```bash
pip install adafruit-circuitpython-bmp3xx
pip install smbus2
pip install adafruit-circuitpython-vl53l1x
pip install adafruit-circuitpython-vl6180x
```
SLAM require:
```bash
sudo apt install libxcb1 libx11-dev libegl1-mesa
export QT_QPA_PLATFORM=xcb
```

