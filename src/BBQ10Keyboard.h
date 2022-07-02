#include <driver/i2c.h>

#define BBQ10KEYBOARD_DEFAULT_ADDR 0x1f
#define BBQ10KEYBOARD_DEFAULT_SDA 23
#define BBQ10KEYBOARD_DEFAULT_SCL 22
#define BBQ10KEYBOARD_DEFAULT_I2C_PORT 0

class BBQ10Keyboard
{
    public:
        enum KeyState
        {
            StateIdle = 0,
            StatePress,
            StateLongPress,
            StateRelease
        };

        struct KeyEvent
        {
            char key;
            KeyState state;
        };

        BBQ10Keyboard();

        void begin(uint8_t addr = BBQ10KEYBOARD_DEFAULT_ADDR,
                   uint8_t pin_sda = BBQ10KEYBOARD_DEFAULT_SDA,
                   uint8_t pin_scl = BBQ10KEYBOARD_DEFAULT_SCL,
                   uint8_t port = BBQ10KEYBOARD_DEFAULT_I2C_PORT);

        void reset(void);

        void attachInterrupt(gpio_num_t pin, gpio_isr_t func) const;
        void detachInterrupt(gpio_num_t pin) const;
        void clearInterruptStatus(void);

        uint8_t status(void) const;
        uint8_t keyCount(void) const;
        KeyEvent keyEvent(void) const;

        float backlight() const;
        void setBacklight(float value);

        float backlight2() const;
        void setBacklight2(float value);

        void pinMode(uint8_t pin, uint8_t mode);
        void digitalWrite(uint8_t pin, uint8_t val);
        int digitalRead(uint8_t pin);

        uint8_t readRegister8(uint8_t reg) const;
        uint16_t readRegister16(uint8_t reg) const;
        uint8_t readRegisterBit(uint8_t reg, uint8_t bit);
        void writeRegister(uint8_t reg, uint8_t value);
        void updateRegisterBit(uint8_t reg, uint8_t bit, uint8_t value);

    private:
        uint8_t m_addr;
        i2c_cmd_handle_t m_cmd;
        uint8_t m_port;
};
