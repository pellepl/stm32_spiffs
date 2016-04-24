## STM32 target playground for spiffs

[spiffs](https://github.com/pellepl/spiffs) integrated on an STM32F103 devboard. The particular one used in this project is the chinese HY-STM32-100P commonly found on ebay or aliexpress. 
This devboard have a 2 megabyte Winbond spiflash mounted which is used for testing spiffs.

This project is basically just a command line interface around spiffs I use for testing new stuff.

Plug your pc to the USB port connected with the PL2102 for getting a USB/UART bridge. Connect using 921600N81 and start playing around. 
If some poor sod but myself will ever use this, try issuing `help all` - it's a good start.

![the devboard](https://raw.githubusercontent.com/pellepl/stm32_spiffs/master/content/hy-stm32-100p.jpg)
