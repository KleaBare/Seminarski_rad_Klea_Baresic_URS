// max30100_simple.h
#define MAX30100_ADDR        0xAE  // 0x57 << 1
#define MAX30100_REG_MODE    0x06
#define MAX30100_REG_SPO2    0x07
#define MAX30100_REG_LED     0x09
#define MAX30100_REG_FIFO    0x05
#define MAX30100_REG_INT_EN  0x01

void MAX30100_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    // Reset
    data = 0x40;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, 0x06, 1, &data, 1, 100);
    HAL_Delay(100);

    // SpO2 mode
    data = 0x03;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, MAX30100_REG_MODE, 1, &data, 1, 100);

    // SpO2 config: 100Hz, 1600us pulse
    data = 0x47;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, MAX30100_REG_SPO2, 1, &data, 1, 100);

    // LED struja: IR=50mA, RED=50mA
    data = 0xFF;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, MAX30100_REG_LED, 1, &data, 1, 100);
}

void MAX30100_ReadFifo(I2C_HandleTypeDef *hi2c, uint16_t *ir, uint16_t *red)
{
    uint8_t buf[4];
    HAL_I2C_Mem_Read(hi2c, MAX30100_ADDR, MAX30100_REG_FIFO, 1, buf, 4, 100);
    *ir  = (buf[0] << 8) | buf[1];
    *red = (buf[2] << 8) | buf[3];
}
