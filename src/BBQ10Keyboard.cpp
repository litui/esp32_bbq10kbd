#include <stdio.h>
#include <soc/i2c_reg.h>
#include <driver/gpio.h>
#include <bitset>
#include <string>
#include "BBQ10Keyboard.h"

namespace bbq10 {

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

void BBQ10Keyboard::begin(uint8_t addr, gpio_num_t pin_sda, gpio_num_t pin_scl, i2c_port_t port)
{
    m_port = port;
    m_addr = addr;
    i2c_config_t conf = { };
    conf.mode = I2C_MODE_MASTER;
    conf.scl_io_num = pin_scl;
    conf.sda_io_num = pin_sda;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL; //Any one clock source that is available for the specified frequency may be choosen

    if (i2c_param_config((i2c_port_t) m_port, &conf) != ESP_OK) {
        ESP_LOGE(TAG, "Error on i2c_param_config.");
        return;
    }
    if (i2c_driver_install(m_port, conf.mode, 0, 0, 0) != ESP_OK) {
        ESP_LOGE(TAG, "Error on i2c_driver_install.");
        return;
    }

    // Use the size of the register used to store timeout bits to determine
    // a reasonable timeout value.  This is very silly.

    #if defined(I2C_TIME_OUT_REG_M)  // Standard on ESP32
        int reg_max = I2C_TIME_OUT_REG_M + 1;
    #elif defined(I2C_TIME_OUT_VALUE_M)  // ESP32 S3, possibly others.
        int reg_max = I2C_TIME_OUT_VALUE_M + 1;
    #else
        int reg_max = 1048576;  // Assume same values as ESP32. Probably bad assumption.
    #endif
    ESP_LOGI(TAG, "Max timeout value for this board: %d", reg_max);

    int timeout = 0;
    i2c_get_timeout((i2c_port_t) m_port, &timeout);
    ESP_LOGI(TAG, "ORIGINAL I2C TIMEOUT: %d", timeout);
    i2c_set_timeout((i2c_port_t) m_port, reg_max / 3 * 2);
    i2c_get_timeout((i2c_port_t) m_port, &timeout);
    ESP_LOGI(TAG, "NEW I2C TIMEOUT: %d", timeout);

    m_buf = (uint8_t*) malloc (sizeof(uint8_t) * 2048);
    reset();
}

void BBQ10Keyboard::reset()
{
    uint8_t rst = _REG_RST;

    // i2c_master_write_to_device(m_port, m_addr, &rst - 1, 1, 10/portTICK_PERIOD_MS);

    
    i2c_cmd_handle_t m_cmd = i2c_cmd_link_create();
    i2c_master_start(m_cmd);
    i2c_master_write_byte(m_cmd, m_addr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(m_cmd, rst, true);
    i2c_master_stop(m_cmd);
    i2c_master_cmd_begin(m_port, m_cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(m_cmd);
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
    ESP_LOGD(TAG, "Called BBQ10Keyboard::keyEvent().");
    KeyEvent event = { .key = '\0', .state = StateIdle };

    if (keyCount() == 0) {
        ESP_LOGD(TAG, "Returned 0 from keyCount()");
        return event;
    }

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
    esp_err_t i2c_err;
    uint8_t len = 1;
    uint8_t r_buf = 0;
    i2c_err = i2c_master_write_read_device(m_port, m_addr, &reg, 1, &r_buf, 1, 10/portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred while reading 8 bit register %02x:%02x: %d", m_addr, reg, i2c_err);
    }
    return r_buf;
}

uint16_t BBQ10Keyboard::readRegister16(uint8_t reg) const
{
    esp_err_t i2c_err;
    uint8_t len = 2;
    uint8_t *l_buf = (uint8_t*) malloc(sizeof(uint8_t)*len);
    i2c_err = i2c_master_write_read_device(m_port, m_addr, &reg, 1, l_buf, 2, 10/portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred while reading 16 bit register %02x:%02x: %d", m_addr, reg, i2c_err);
    }
    
    uint16_t lr_buf = (l_buf[1] << 8) | l_buf[0];
    free(l_buf);
    return lr_buf;
}

uint8_t BBQ10Keyboard::readRegisterBit(uint8_t reg, uint8_t bit)
{
    return ((readRegister8(reg) >> (bit)));
}

void BBQ10Keyboard::writeRegister(uint8_t reg, uint8_t value)
{
    esp_err_t i2c_err;
    uint8_t len = 2;
    uint8_t *w_buf = (uint8_t*) malloc (sizeof(uint8_t) * 2);
    w_buf[0] = reg | _WRITE_MASK;
    w_buf[1] = value;
    i2c_err = i2c_master_write_to_device(m_port, m_addr, w_buf, 2, 10/portTICK_PERIOD_MS);
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred while writing %02x:%02x: %d", m_addr, reg, i2c_err);
    }
    free(w_buf);
}

void BBQ10Keyboard::updateRegisterBit(uint8_t reg, uint8_t bit, uint8_t value)
{
    uint8_t oldValue = readRegister8(reg);
    uint8_t newValue = oldValue;

    ((value) ? ((newValue) |= (1UL << (bit))) : ((newValue) &= ~(1UL << (bit))));
    
    if (newValue != oldValue)
        writeRegister(reg, newValue);
}

} // namespace bbq10
