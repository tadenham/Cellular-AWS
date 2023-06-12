# Cellular_AWS

ESP-IDF project for cellular automated weather stations.

Currently, this example takes temperature, humidity, and dew point measurements every 10 seconds using a SHT85 sensor. A pulse signal from a rain gauge is handled by the ULP.

Data is transmitted every 5-minutes via a SIMCom SIM7600 modem. Transmitted temperature, humidity, and dew point values are 5-minute averages.

This example also allows transmission via ESP-Now instead of cellular when it close proximity to a building.
