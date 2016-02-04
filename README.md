# TinyTX-TinyRX-Project

This programm and project is based on the awesome work of Nathan Chantrell
* https://nathan.chantrell.net/tag/tinytx/
* https://github.com/nathanchantrell/TinyTX
and was deployed on the TinyTX4 board by meigrafd
* https://github.com/meigrafd/TinyTX4
 Also inspired by the emonTH project
* https://github.com/openenergymonitor/emonTH
other references in the code

Features:
*  Support of one or two DHT22 on one sender
*  Send temp and humidity on different intervals than battery voltage
*  Read battery and vcc voltage using higher precision interrupt method
*  For battery connection I refered to: 
*  http://jeelabs.org/2013/05/16/measuring-the-battery-without-draining-it/index.html
*  Only send when values have changed to save battery
*  Random backoff timer when ACKs not received for each sender
*  Send frames contain ack error count, and dht22 read error count
*  Use a stripped down version of SoftwareSerial for read only to save a pin for RX
*  and memory on the attiny
*  Battery protection: shuts down when voltage is low to prevent leakage

The link from the received bytes on serial to OpenHab is done by a python
programm located in  "processing scripts"

"tinySender" contains the sender arduino sketch
"tinyReveiver" the receiver sketch
"SendOnlySoftwareSerial" is a stripped down version of the original Arduino library
See the JPG file for a glimpse of the TinyTX4 board with the battery level reading
 "circuit".
