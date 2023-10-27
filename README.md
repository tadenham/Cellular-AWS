# Cellular AWS

ESP-IDF project for cellular automated weather stations.

Currently, this example takes temperature, humidity, and dew point measurements every 10 seconds using a SHT3x sensor. A pulse signal from a rain gauge is handled by the ULP. Barometric pressure is measured from a BMP280 sensor at 15-minute intervals.

Data is transmitted every 15-minutes via a SIMCom SIM7600 modem. Transmitted temperature, humidity, and dew point values are 5-minute averages.
