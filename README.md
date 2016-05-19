# AIS-repeater
AIS receiver repeater - complements lora-gateway repository
target board is Teensy 3.1 but should work on other Arduino compatible boards too if you modify the sleep library
(teensyLC should be ok for example)

compile code onto suitable microcontroller - needs 1xSerial, SPI, 1xADC (12 bit defined in software, could be redesigned for lower) and a suitable sleep library for low power modde

To keep power consumption down, run at low clock speed eg 24MHz or lower. See pjrc.com for compatibility issues running at various frequ

NEEDS SLEEP library - see separate folder. Add library to your Arduino IDE before compiling


