.. _energy_sensor:

Energy Sensor: 
##########################


Energy Sensor is a NRF52833 code to read a GPS and the Temperature. This data is transmitted through Lorawan using the Helium Network

I am using the module Ebyte E73-2G4M08S1E nRF52833 with an external 32KHz crystal.

The Lorawan Module is SX1276 (RFM95) based. The GPS module is a Quectel LC86

There is a special routine to calculate the FFT, works but it's disabled because fist I am finishing the GPS routine.