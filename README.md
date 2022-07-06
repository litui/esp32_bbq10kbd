# BBQ10Keyboard Library for ESP32

ESP32 library for interfacing the BB Q10 and BB Q20 Keyboards over I2C.

_This library works a touch differently from the Arduino library._ 
Be sure to look over the code as the examples are not yet ported.

The firmwares that this library should be used with can be found here: 

* BB Q10: https://github.com/solderparty/bbq10kbd_i2c_sw
* BB Q20: https://github.com/solderparty/i2c_puppet

# Installation

Within your ESP-IDF project directory (project must be git-based):

```
git submodule add https://github.com/litui/esp32_bbq10kbd components/BBQ10Keyboard
```

Alternately, download the zip of this repo and unzip it to components/BBQ10Keyboard.

Be sure to add BBQ10Keyboard to your CMakeLists.txt requirements for your main application if needed.

# TODO

* Examples still need to be ported.
* Fully test the GPIO expander functionality.