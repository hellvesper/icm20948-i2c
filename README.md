# icm20948-i2c
ESP-IDF ICM20948 I2C driver ![icm20948-i2c-main](https://github.com/hellvesper/icm20948-i2c/actions/workflows/cmake-esp-idf-platform.yml/badge.svg?branch=main)

This is Invensense ICM20x48 family official driver wrapper and port to ESP32 (ESP-IDF v.4). 

**Using this driver you agreed with Invensense License**

Maximum rate using DMP is 225Hz according to documentation, 227Hz in practice. So it's not necessary to use SPI. SPI support persist in driver but not tested.

Check Pyhon Demo visualisation in `components/icm20948/visualise_dmp.py`
* To run demo create venv via `python3 -m venv venv`
* activate venv `source venv/bin/activate`
* install requirements packages `open3d`, `numpy-quanterion` and `pyserial` using `pip install -r requirements.txt`
* set your port name and baud rate in `visualise_dmp.py` init section
* cd into directory where `visualise_dmp.py` located and run `python visualise_dmp.py`
Note blue axis is Y (green and red is X and Z) and initial scene position is upside down but plane points in right direction, you can drag zoom and rotate scene by mouse



https://github.com/hellvesper/icm20948-i2c/assets/2809330/4a16d812-1839-4585-b0a8-e2ce4b5e4db5

