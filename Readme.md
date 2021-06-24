# Automatic Lighthouse System (ALS)

## Introduction

This project is an exercise to improve Arduino skills in programing a self sufficient unit.<br>
The project is inspired by [Soviet-era RTG-powered automatic lighthouses]("https://www.bbc.com/reel/embed/p0931jtk") placed on northen siberian shore.

This works is based on Arduino Mega2560, more detailed documentation is provided about both code and opertions.

## Capabilities
ALS is mostly autonomous in its operation: user interaction is required only when collecting gathered data or when resetting the unit due to its relocation.<br>The unit is capable of tolerating various abnormalities without freezing.  
List of current ALS features:

* Automatic clock sync from **GPS** satellites
* Battery operated **RTC** as time signal backup
* _9 V_ to _40 V_ input voltage thanks to internal linear voltage regulator
* **Bluetooth**wireless serial interface for data exchange and programming
* Quick reporting of weather data via LCD
* Storing of weather data on **SD**
* Programmable **LED** lightning
* Remote software reboot
* Download of file via Serial
* Configurable settings via dedicated file on SD memory
## User guide
A dedicated pdf file illustrates how to operate and program the ALS via bluetooth wireless interface.

## Under development
* Upload of file via Serial
* Energy saving mode
* User-friendly GUI
* Code in-depth documentation