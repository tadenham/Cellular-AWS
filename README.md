# Cellular AWS

ESP-IDF project for cellular automated weather stations.

This example takes temperature, humidity, and dew point measurements every 10 seconds using a SHT3x or HDC3020 sensor. Pulse signals from the rain gauge and anemometer is handled by the ULP, while an external ADC is used to measure the wind direction from a AS5600 sensor. Barometric pressure is measured from a BMP280 sensor at each transmission interval.

Data is transmitted at a configurable interval via a SIMCom SIM7070 or SIM7600 modem. Transmitted temperature, humidity, and dew point values are instantaneous values or (optionally) 5-minute averages.
