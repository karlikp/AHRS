# Attitude and Heading Reference System for a smart valet
I take part in project "Developing a smart valet parking system that utilizes optimal space". where I'm responsible for integrate  

## Description
![image](https://github.com/user-attachments/assets/11971815-8101-4355-8803-93c04207fe87)


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

Creating python virtual environment(if you don't have):
```bash
python3 -m venv your_env
```

activate the virtual environent:
```bash
source your_env/bin/activate
```

If you don't have the libraries, you should install them:
```bash
pip install adafruit-circuitpython-bmp3xx
pip install smbus2
pip install adafruit-circuitpython-vl53l1x
pip install adafruit-circuitpython-vl6180x
```

