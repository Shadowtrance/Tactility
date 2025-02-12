#pragma once

// Display backlight (PWM)
#define JC2432W328C_LCD_BACKLIGHT_LEDC_TIMER LEDC_TIMER_0
#define JC2432W328C_LCD_BACKLIGHT_LEDC_MODE LEDC_LOW_SPEED_MODE
#define JC2432W328C_LCD_BACKLIGHT_LEDC_CHANNEL LEDC_CHANNEL_0
#define JC2432W328C_LCD_BACKLIGHT_LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define JC2432W328C_LCD_BACKLIGHT_LEDC_FREQUENCY (1000)

#define JC2432W328C_LCD_PIN_BACKLIGHT GPIO_NUM_27

// Display
#define JC2432W328C_LCD_SPI_HOST SPI2_HOST
#define JC2432W328C_LCD_HORIZONTAL_RESOLUTION 240
#define JC2432W328C_LCD_VERTICAL_RESOLUTION 320
#define JC2432W328C_LCD_BITS_PER_PIXEL 16
#define JC2432W328C_LCD_DRAW_BUFFER_HEIGHT (JC2432W328C_LCD_VERTICAL_RESOLUTION / 10)
#define JC2432W328C_LCD_DRAW_BUFFER_SIZE (JC2432W328C_LCD_HORIZONTAL_RESOLUTION * JC2432W328C_LCD_DRAW_BUFFER_HEIGHT * (JC2432W328C_LCD_BITS_PER_PIXEL / 8))
#define JC2432W328C_LCD_PIN_CS GPIO_NUM_15
#define JC2432W328C_LCD_PIN_DC GPIO_NUM_2

