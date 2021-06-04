# Automatic Lighthouse System (ALS)

## Introduction

This project is an exercise to improve Arduino skills in programing a self sufficient unit.<br>
The project is inspired by [Soviet-era RTG-powered automatic lighthouses]("https://www.bbc.com/reel/embed/p0931jtk") placed on northen siberian shore.

This works is based on Arduino Mega2560, more detailed documentation is provided about both code and opertions.

## Capabilities
ALS is mostly autonomous in its operation: user interaction is required only when collecting gathered data or when resetting the unit due to its relocation.<br>The unit is capable of tolerating various abnormalities without freezing.  
A brief list of ALS features:

1. Automatic clock sync from <span style="color:lime">**GPS**</span> satellites
2. Battery operated <span style="color:cyan">**RTC**</span> as time signal backup
3. _9 V_ to _40 V_ input voltage thanks to internal linear voltage regulator
4. <span style="color:blue">**Bluetooth**</span> wireless serial interface for data exchange and programming
5. Quick reporting of weather data via LCD
6. Storing of weather data on SD
7. Programmable LED lightning

## User guide
A dedicated pdf file illustrates how to operate and program the ALS via bluetooth wireless interface.

## Code documentation
Code documentation is available within the code and in dedicated pdf documentation, created via Doxygen.