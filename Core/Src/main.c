#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h> // Added for memset
#define LCD_ADDR (0x27 << 1)
#define LCD_RS  0x01
#define LCD_EN  0x04
#define LCD_BL  0x08
#define LCD_COMMAND 0x00
#define LCD_DATA    0x01
#define LCD_CLEAR           0x01
#define LCD_FUNCTION_SET    0x28
#define LCD_DISPLAY_ON      0x0C
#define LCD_ENTRY_MODE      0x06
#define TRIG_PIN 0
#define ECHO_PIN 1
#define LED_PIN 8      // PA8 cho LED nháy
#define GREEN_LED_PIN 9

uint32_t Difference = 0;
uint8_t Distance = 0;

// Prototypes
void initI2C(void);
void initGPIO(void);
void initLCD(void);
void writeLCD(uint8_t data, uint8_t mode);
void sendCmdToLCD(uint8_t cmd);
void printLCD(const char* str);
void initTimer2(void);
void delayUs(uint16_t time);
void delayMs(uint32_t time);
uint8_t getDistance(void);
void updateLCDLine(uint8_t line, const char* str); // Helper function

int main(void) {
    initGPIO();
    initI2C();
    initTimer2();
    initLCD();

    // Khởi tạo màn hình LCD với hai dòng
    sendCmdToLCD(LCD_CLEAR);
    updateLCDLine(1, "Distance:    "); // First line
    updateLCDLine(2, "Object: 0");      // Second line initially no object

    // Tắt LED nháy ban đầu
    GPIOA->BSRR = (1 << (LED_PIN + 16)); // Tắt LED (Active high)

    while (1) {
        Distance = getDistance();
        char buffer[16];

        // Update Distance on first line
        if (Distance == 0xFF) {
            snprintf(buffer, sizeof(buffer), "Distance: Err");
        } else {
            snprintf(buffer, sizeof(buffer), "Distance:%3dcm", Distance);
        }
        updateLCDLine(1, buffer);

        // Determine Object Detection Status
        uint8_t object = (Distance <= 30 && Distance != 0xFF) ? 1 : 0;

        // Update Object status on second line
        snprintf(buffer, sizeof(buffer), "Object: %d", object);
        updateLCDLine(2, buffer);

        if (object) {
            uint8_t freq;
            if (Distance == 0) {
                freq = 10; // Cực gần
            } else {
                freq = 1 + ((30 - Distance) * 9) / 30;
                if (freq > 10) freq = 10;
            }
            uint32_t delay_time_ms = 50 / freq; // Half period = 500ms / freq
            GPIOA->ODR ^= (1 << LED_PIN);
            delayMs(delay_time_ms);
        } else {
            GPIOA->BSRR = (1 << (LED_PIN + 16)); // Tắt LED
            delayMs(100);
        }
    }
}

void initGPIO(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;
    GPIOB->MODER = (GPIOB->MODER & ~(0xF << 16)) | (0xA << 16); // PB8 và PB9 là AF4
    GPIOB->OTYPER |= (1 << 8) | (1 << 9); // Open-Drain
    GPIOB->OSPEEDR |= (0xF << 16); // Tốc độ cao
    GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL8_Pos) | (4 << GPIO_AFRH_AFSEL9_Pos); // AF4 cho I2C1
    GPIOA->MODER = (GPIOA->MODER & ~0xF) | 0x1; // PA0 là Output, PA1 là Input
    GPIOA->OTYPER &= ~(1 << TRIG_PIN); // Push-Pull cho TRIG_PIN
    GPIOA->OSPEEDR |= (3 << (TRIG_PIN * 2)); // Tốc độ cao
    GPIOA->PUPDR &= ~0xF; // No Pull-Up, No Pull-Down
    GPIOA->MODER |= (0x01 << (LED_PIN * 2)); // PA8 là Output
    GPIOA->OTYPER &= ~(1 << LED_PIN); // Push-Pull
    GPIOA->OSPEEDR |= (0x03 << (LED_PIN * 2)); // Tốc độ cao
    GPIOA->PUPDR &= ~(0x03 << (LED_PIN * 2)); // No Pull-Up, No Pull-Down
}

void initI2C(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR2 = 16; // Assuming APB1 clock is 16MHz
    I2C1->CCR = 80; // 100kHz
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

void initTimer2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15;
    TIM2->ARR = 0xFFFF;
    TIM2->CR1 = 1;
}

void delayUs(uint16_t time) {
    TIM2->CNT = 0;
    while (TIM2->CNT < time);
}

void delayMs(uint32_t time) {
    for (uint32_t i = 0; i < (time * 16000); i++) {
        __NOP();
    }
}

uint8_t getDistance(void) {
    GPIOA->BSRR = (1 << TRIG_PIN);
    delayUs(10);
    GPIOA->BSRR = (1 << (TRIG_PIN + 16));
    uint32_t timeout = 0;
    while (!(GPIOA->IDR & (1 << ECHO_PIN))) {
        if (++timeout > 30000) return 0xFF;
    }
    TIM2->CNT = 0;
    while (GPIOA->IDR & (1 << ECHO_PIN)) {
        if (TIM2->CNT > 30000) break;
    }
    Difference = TIM2->CNT;
    if (Difference == 0xFFFF || Difference > 30000) return 0xFF;
    uint16_t distance = (Difference * 34) / 2000;
    if (distance > 400) distance = 400;
    return (uint8_t)distance;
}

void writeLCD(uint8_t data, uint8_t mode) {
    uint8_t high = data & 0xF0;
    uint8_t low = (data << 4) & 0xF0;
    uint8_t commands[] = {
        high | mode | LCD_BL | LCD_EN,
        high | mode | LCD_BL,
        low | mode | LCD_BL | LCD_EN,
        low | mode | LCD_BL
    };
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    I2C1->DR = LCD_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    for (int i = 0; i < 4; ++i) {
        I2C1->DR = commands[i];
        while (!(I2C1->SR1 & I2C_SR1_BTF));
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    for (volatile int i = 0; i < 1600; i++);
}

void sendCmdToLCD(uint8_t cmd) {
    writeLCD(cmd, LCD_COMMAND);
}

void initLCD(void) {
    for (volatile int i = 0; i < 80000; i++);
    sendCmdToLCD(0x33);
    for (volatile int i = 0; i < 8000; i++);
    sendCmdToLCD(0x32);
    sendCmdToLCD(LCD_FUNCTION_SET);
    sendCmdToLCD(LCD_DISPLAY_ON);
    sendCmdToLCD(LCD_CLEAR);
    sendCmdToLCD(LCD_ENTRY_MODE);
}

void printLCD(const char* str) {
    while (*str) {
        writeLCD(*str++, LCD_DATA);
    }
}

void updateLCDLine(uint8_t line, const char* str) {
    if (line == 1) {
        sendCmdToLCD(0x80);
    } else if (line == 2) {
        sendCmdToLCD(0xC0);
    }
    printLCD(str);
}
