At present, data_logger.ino is Arduino code for an ESP32 pico kit v4.1.
It will listen for data from collars and send it to the serial port.
It is quite convenient to carry it connected to a laptop.
With very minor modifications, it may store aggregated data in a SD card.
In any case, please check your MACs.

Usage for a logger ESP32 linke by serial USB to a laptop: 
Linux: (stty raw 115200;cat > tmp.log) < /dev/ttyUSB1
