#include <driver/gpio.h>
#include <driver/i2c.h>
#include "BBQ10Keyboard.h"

// Standins for leftover Arduino defines
#define LOW    0x00
#define HIGH   0x01
//GPIO FUNCTIONS
#define INPUT             0x01
// Changed OUTPUT from 0x02 to behave the same as Arduino pinMode(pin,OUTPUT) 
// where you can read the state of pin even when it is set as OUTPUT
#define OUTPUT            0x03 
#define PULLUP            0x04
#define INPUT_PULLUP      0x05
#define PULLDOWN          0x08
#define INPUT_PULLDOWN    0x09
#define OPEN_DRAIN        0x10
#define OUTPUT_OPEN_DRAIN 0x12
#define ANALOG            0xC0

#define _REG_VER 0x01 // fw version
#define _REG_CFG 0x02 // config
#define _REG_INT 0x03 // interrupt status
#define _REG_KEY 0x04 // key status
#define _REG_BKL 0x05 // backlight
#define _REG_DEB 0x06 // debounce cfg
#define _REG_FRQ 0x07 // poll freq cfg
#define _REG_RST 0x08 // reset
#define _REG_FIF 0x09 // fifo
#define _REG_BK2 0x0A // backlight 2
#define _REG_DIR 0x0B // gpio direction
#define _REG_PUE 0x0C // gpio input pull enable
#define _REG_PUD 0x0D // gpio input pull direction
#define _REG_GIO 0x0E // gpio value
#define _REG_GIC 0x0F // gpio interrupt config
#define _REG_GIN 0x10 // gpio interrupt status

#define _WRITE_MASK (1 << 7)

#define CFG_OVERFLOW_ON  (1 << 0)
#define CFG_OVERFLOW_INT (1 << 1)
#define CFG_CAPSLOCK_INT (1 << 2)
#define CFG_NUMLOCK_INT  (1 << 3)
#define CFG_KEY_INT      (1 << 4)
#define CFG_PANIC_INT    (1 << 5)

#define INT_OVERFLOW     (1 << 0)
#define INT_CAPSLOCK     (1 << 1)
#define INT_NUMLOCK      (1 << 2)
#define INT_KEY          (1 << 3)
#define INT_PANIC        (1 << 4)

#define KEY_CAPSLOCK     (1 << 5)
#define KEY_NUMLOCK      (1 << 6)
#define KEY_COUNT_MASK   (0x1F)

#define DIR_OUTPUT       0
#define DIR_INPUT        1

#define PUD_DOWN         0
#define PUD_UP           1

BBQ10Keyboard::BBQ10Keyboard()
{
}

void BBQ10Keyboard::begin(uint8_t addr, uint8_t pin_sda, uint8_t pin_scl, uint8_t port)
{
    m_port = 0;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pin_sda,         // select GPIO specific to your project
        .scl_io_num = pin_scl,         // select GPIO specific to your project
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 400000
        },  // select frequency specific to your project
        .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    i2c_param_config(m_port, &conf);
    m_addr = addr;
    i2c_driver_install(m_port, I2C_MODE_MASTER, 0, 0, 0);
    m_cmd = i2c_cmd_link_create();
    reset();
}

void BBQ10Keyboard::reset()
{
    i2c_master_start(m_cmd);
    i2c_master_write_byte(m_cmd, m_addr, true);
    i2c_master_write_byte(m_cmd, _REG_RST, true);
    i2c_master_stop(m_cmd);
    i2c_master_cmd_begin(m_port, m_cmd, 200);
}

void BBQ10Keyboard::attachInterrupt(gpio_num_t pin, gpio_isr_t func) const
{
    // Initialize ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_config_t gpioconf = {
        .pin_bit_mask = (uint64_t) 1 << pin,          /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        .mode         = GPIO_MODE_INPUT,              /*!< GPIO mode: set input/output mode                     */
        .pull_up_en   = GPIO_PULLUP_ENABLE,           /*!< GPIO pull-up                                         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,        /*!< GPIO pull-down                                       */
        .intr_type    = GPIO_INTR_LOW_LEVEL           /*!< GPIO interrupt type                                  */
    };
    gpio_config(&gpioconf);
	gpio_isr_handler_add(pin, func, 0);
}

void BBQ10Keyboard::detachInterrupt(gpio_num_t pin) const
{
    gpio_isr_handler_remove(pin);
}

void BBQ10Keyboard::clearInterruptStatus()
{
    writeRegister(_REG_INT, 0x00);
}

uint8_t BBQ10Keyboard::status() const
{
    return readRegister8(_REG_KEY);
}

uint8_t BBQ10Keyboard::keyCount() const
{
    return status() & KEY_COUNT_MASK;
}

BBQ10Keyboard::KeyEvent BBQ10Keyboard::keyEvent() const
{
    KeyEvent event = { .key = '\0', .state = StateIdle };

    if (keyCount() == 0)
        return event;

    const uint16_t buf = readRegister16(_REG_FIF);
    event.key = buf >> 8;
    event.state = KeyState(buf & 0xFF);

    return event;
}

float BBQ10Keyboard::backlight() const
{
    return readRegister8(_REG_BKL) / 255.0f;
}

void BBQ10Keyboard::setBacklight(float value)
{
    writeRegister(_REG_BKL, value * 255);
}

float BBQ10Keyboard::backlight2() const
{
    return readRegister8(_REG_BK2) / 255.0f;
}

void BBQ10Keyboard::setBacklight2(float value)
{
    writeRegister(_REG_BK2, value * 255);
}

void BBQ10Keyboard::pinMode(uint8_t pin, uint8_t mode)
{
    if (pin > 7)
        return;

    if (mode == INPUT || mode == INPUT_PULLUP) {
        updateRegisterBit(_REG_DIR, pin, DIR_INPUT);
    } else if (mode == GPIO_MODE_OUTPUT) {
        updateRegisterBit(_REG_DIR, pin, DIR_OUTPUT);
    }
}

void BBQ10Keyboard::digitalWrite(uint8_t pin, uint8_t val)
{
    if (pin > 7)
        return;

    if (readRegisterBit(_REG_DIR, pin) == DIR_INPUT) {
        updateRegisterBit(_REG_PUD, pin, val == LOW ? PUD_DOWN : PUD_UP);
    } else {
        updateRegisterBit(_REG_GIO, pin, val == HIGH);
    }
}

int BBQ10Keyboard::digitalRead(uint8_t pin)
{
    if (pin > 7)
        return LOW;

    if (readRegisterBit(_REG_GIO, pin))
        return HIGH;

    return LOW;
}

uint8_t BBQ10Keyboard::readRegister8(uint8_t reg) const
{
    i2c_master_start(m_cmd);
    i2c_master_write_byte(m_cmd, reg, true);
    i2c_master_stop(m_cmd);
    i2c_master_cmd_begin(m_port, m_cmd, 200);
    uint8_t *r_buf = (uint8_t*) malloc(1);
    i2c_master_read_from_device(m_port, m_addr, r_buf, sizeof(r_buf), 200);
    return *r_buf;
}

uint16_t BBQ10Keyboard::readRegister16(uint8_t reg) const
{
    i2c_master_start(m_cmd);
    i2c_master_write_byte(m_cmd, reg, true);
    i2c_master_stop(m_cmd);
    i2c_master_cmd_begin(m_port, m_cmd, 200);
    uint8_t *r_buf = (uint8_t*) malloc(2);
    r_buf[0] = 0;
    r_buf[1] = 0;
    i2c_master_read_from_device(m_port, m_addr, r_buf, sizeof(r_buf), 200);
    if (!r_buf[0] or !r_buf[1]) {
        return 0;
    }
    return (r_buf[1] << 8) | r_buf[0];
}

uint8_t BBQ10Keyboard::readRegisterBit(uint8_t reg, uint8_t bit)
{
    return ((readRegister8(reg) >> (bit)) & 0x01);
}

void BBQ10Keyboard::writeRegister(uint8_t reg, uint8_t value)
{
    const uint8_t *w_buf = (uint8_t*) malloc (1);
    w_buf = &value;
    i2c_master_write_to_device(m_port, m_addr, w_buf, 1, 200);
}

void BBQ10Keyboard::updateRegisterBit(uint8_t reg, uint8_t bit, uint8_t value)
{
    uint8_t oldValue = readRegister8(reg);
    uint8_t newValue = oldValue;

    ((value) ? ((newValue) |= (1UL << (bit))) : ((newValue) &= ~(1UL << (bit))));
    
    if (newValue != oldValue)
        writeRegister(reg, newValue);
}
