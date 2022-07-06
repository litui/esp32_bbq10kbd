#include "esp_log.h"
#include "driver/i2c.h"

namespace bbq10 {

static const uint8_t BBQ10KEYBOARD_DEFAULT_ADDR = 0x1f;
static const gpio_num_t BBQ10KEYBOARD_DEFAULT_SDA = GPIO_NUM_21;
static const gpio_num_t BBQ10KEYBOARD_DEFAULT_SCL = GPIO_NUM_22;
static const i2c_port_t BBQ10KEYBOARD_DEFAULT_I2C_PORT = I2C_NUM_0;

static const char* TAG = "BBQ10Keyboard";

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
                   gpio_num_t pin_sda = BBQ10KEYBOARD_DEFAULT_SDA,
                   gpio_num_t pin_scl = BBQ10KEYBOARD_DEFAULT_SCL,
                   i2c_port_t port = BBQ10KEYBOARD_DEFAULT_I2C_PORT);

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
        uint8_t m_port;
        uint8_t *m_buf;
        uint32_t m_bufsize;
};

} // namespace bbq10