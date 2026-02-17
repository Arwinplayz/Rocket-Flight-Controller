#include "stm32f7xx_hal.h"
#include <math.h>

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;

#define IMU_I2C_ADDR (0x68 << 1)
#define IMU_REG_WHO_AM_I 0x75
#define IMU_REG_PWR_MGMT 0x06
#define IMU_REG_ACCEL_XOUT_H 0x2D
#define IMU_REG_GYRO_XOUT_H 0x33

#define SERVO_TIMER htim3
#define SERVO_CHANNEL TIM_CHANNEL_1
#define SERVO_PWM_FREQ_HZ 50.0f
#define SERVO_MIN_PULSE_US 1000.0f
#define SERVO_MAX_PULSE_US 2000.0f

#define LOOP_DT_S 0.002f
#define COMPLEMENTARY_ALPHA 0.98f

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);

static void IMU_Init(void);
static void IMU_ReadRaw(int16_t *ax, int16_t *az, int16_t *gx);
static float map_pitch_to_pulse(float pitch_deg);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    IMU_Init();
    HAL_TIM_PWM_Start(&SERVO_TIMER, SERVO_CHANNEL);

    float pitch_deg = 0.0f;

    while (1)
    {
        static uint32_t last_tick = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_tick < (uint32_t)(LOOP_DT_S * 1000.0f)) continue;
        last_tick = now;

        int16_t ax_raw, az_raw, gx_raw;
        IMU_ReadRaw(&ax_raw, &az_raw, &gx_raw);

        float ax = (float)ax_raw / 16384.0f;
        float az = (float)az_raw / 16384.0f;
        float gx = (float)gx_raw / 131.0f;

        float pitch_acc = atan2f(-ax, az) * 180.0f / (float)M_PI;
        float pitch_gyro = pitch_deg + gx * LOOP_DT_S;
        pitch_deg = COMPLEMENTARY_ALPHA * pitch_gyro + (1.0f - COMPLEMENTARY_ALPHA) * pitch_acc;

        float pulse_us = map_pitch_to_pulse(pitch_deg);

        uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2;
        float timer_freq = 1000000.0f;
        uint32_t period = (uint32_t)(timer_freq / SERVO_PWM_FREQ_HZ) - 1;
        uint32_t compare = (uint32_t)((pulse_us / 1000000.0f) * timer_freq);

        __HAL_TIM_SET_AUTORELOAD(&SERVO_TIMER, period);
        __HAL_TIM_SET_COMPARE(&SERVO_TIMER, SERVO_CHANNEL, compare);
    }
}

static void IMU_WriteReg(uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void IMU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

static void IMU_Init(void)
{
    HAL_Delay(100);
    IMU_WriteReg(IMU_REG_PWR_MGMT, 0x00);
    HAL_Delay(10);
}

static void IMU_ReadRaw(int16_t *ax, int16_t *az, int16_t *gx)
{
    uint8_t buf[12];
    IMU_ReadRegs(IMU_REG_ACCEL_XOUT_H, buf, 12);

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
}

static float map_pitch_to_pulse(float pitch_deg)
{
    if (pitch_deg < -45.0f) pitch_deg = -45.0f;
    if (pitch_deg > 45.0f) pitch_deg = 45.0f;
    float t = (pitch_deg + 45.0f) / 90.0f;
    return SERVO_MIN_PULSE_US + t * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    HAL_PWREx_EnableOverDrive();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
}

static void MX_I2C1_Init(void)
{
    __HAL_RCC_I2C1_CLK_ENABLE();

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x20404768;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}

static void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2;
    uint32_t prescaler = (timer_clk / 1000000U) - 1U;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = prescaler;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 20000 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SERVO_CHANNEL);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}