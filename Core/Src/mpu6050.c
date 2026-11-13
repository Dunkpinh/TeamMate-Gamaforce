/*
 * mpu6050.c
 * Driver MPU6050 untuk STM32 (HAL) - via I2C
 * Taruh di: Core/Src/mpu6050.c
 */
#include "mpu6050.h"
#include <math.h>

#define RAD_TO_DEG   57.2957795f
#define COMP_ALPHA   0.98f      /* bobot gyro pada complementary filter */

static HAL_StatusTypeDef MPU6050_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, MPU6050_I2C_TIMEOUT);
}

static HAL_StatusTypeDef MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, MPU6050_I2C_TIMEOUT);
}

/* Tulis register + catat langkah/status untuk diagnosa */
static HAL_StatusTypeDef MPU6050_Step(I2C_HandleTypeDef *hi2c, MPU6050_t *dev,
                                      uint8_t step, uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef st;

    dev->init_step = step;
    st = MPU6050_WriteReg(hi2c, reg, val);
    dev->last_status = st;
    return st;
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_t *dev)
{
    HAL_StatusTypeDef st;

    dev->gx_off = dev->gy_off = dev->gz_off = 0.0f;
    dev->roll = dev->pitch = 0.0f;
    dev->angle_init = 0;
    dev->who_am_i = 0;
    dev->init_step = MPU_STEP_NONE;
    dev->last_status = HAL_OK;

    /* 1. Cek apakah device ada di bus (butuh ACK) */
    dev->init_step = MPU_STEP_PROBE;
    st = HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 3, MPU6050_I2C_TIMEOUT);
    dev->last_status = st;
    if (st != HAL_OK) return st;

    /* 2. Baca WHO_AM_I */
    dev->init_step = MPU_STEP_WHOAMI_READ;
    st = MPU6050_ReadRegs(hi2c, MPU6050_REG_WHO_AM_I, &dev->who_am_i, 1);
    dev->last_status = st;
    if (st != HAL_OK) return st;

    /* 3. WHO_AM_I hanya dicatat, TIDAK dipakai untuk menolak.
       MPU6050 asli = 0x68, tapi banyak modul clone/kompatibel memberi nilai lain
       (mis. 0x70, 0x72, 0x74, 0x98) dengan register map yang sama.
       Validasi sebenarnya dilakukan lewat data (az ~ 1 g saat diam). */
    dev->init_step = MPU_STEP_WHOAMI_VALUE;

    /* 4. Reset device */
    st = MPU6050_Step(hi2c, dev, MPU_STEP_RESET, MPU6050_REG_PWR_MGMT_1, 0x80);
    if (st != HAL_OK) return st;
    HAL_Delay(100);

    /* 5. Bangunkan dari sleep, clock = PLL gyro X */
    st = MPU6050_Step(hi2c, dev, MPU_STEP_WAKE, MPU6050_REG_PWR_MGMT_1, 0x01);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    /* 6. DLPF = 3 -> ~44 Hz */
    st = MPU6050_Step(hi2c, dev, MPU_STEP_DLPF, MPU6050_REG_CONFIG, 0x03);
    if (st != HAL_OK) return st;

    /* 7. Sample rate = 1kHz / (1 + 4) = 200 Hz */
    st = MPU6050_Step(hi2c, dev, MPU_STEP_SMPLRT, MPU6050_REG_SMPLRT_DIV, 0x04);
    if (st != HAL_OK) return st;

    /* 8. Gyro +-250 dps */
    st = MPU6050_Step(hi2c, dev, MPU_STEP_GYRO_CFG, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (st != HAL_OK) return st;

    /* 9. Accel +-2 g */
    st = MPU6050_Step(hi2c, dev, MPU_STEP_ACCEL_CFG, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (st != HAL_OK) return st;

    dev->last_tick = HAL_GetTick();
    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *dev)
{
    uint8_t buf[14];
    HAL_StatusTypeDef st = MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (st != HAL_OK) return st;

    /* Urutan register: AX AY AZ TEMP GX GY GZ (masing-masing 2 byte, big-endian) */
    dev->ax_raw   = (int16_t)(buf[0]  << 8 | buf[1]);
    dev->ay_raw   = (int16_t)(buf[2]  << 8 | buf[3]);
    dev->az_raw   = (int16_t)(buf[4]  << 8 | buf[5]);
    dev->temp_raw = (int16_t)(buf[6]  << 8 | buf[7]);
    dev->gx_raw   = (int16_t)(buf[8]  << 8 | buf[9]);
    dev->gy_raw   = (int16_t)(buf[10] << 8 | buf[11]);
    dev->gz_raw   = (int16_t)(buf[12] << 8 | buf[13]);

    dev->ax = dev->ax_raw / MPU6050_ACCEL_SCALE;
    dev->ay = dev->ay_raw / MPU6050_ACCEL_SCALE;
    dev->az = dev->az_raw / MPU6050_ACCEL_SCALE;

    dev->gx = dev->gx_raw / MPU6050_GYRO_SCALE - dev->gx_off;
    dev->gy = dev->gy_raw / MPU6050_GYRO_SCALE - dev->gy_off;
    dev->gz = dev->gz_raw / MPU6050_GYRO_SCALE - dev->gz_off;

    /* Rumus dari datasheet */
    dev->temp = dev->temp_raw / 340.0f + 36.53f;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *dev, uint16_t samples)
{
    float sx = 0, sy = 0, sz = 0;
    uint16_t ok = 0;

    dev->gx_off = dev->gy_off = dev->gz_off = 0.0f;

    for (uint16_t i = 0; i < samples; i++) {
        if (MPU6050_Read_All(hi2c, dev) == HAL_OK) {
            sx += dev->gx;
            sy += dev->gy;
            sz += dev->gz;
            ok++;
        }
        HAL_Delay(5);   /* 200 Hz, sama dengan sample rate */
    }

    if (ok == 0) return HAL_ERROR;

    dev->gx_off = sx / ok;
    dev->gy_off = sy / ok;
    dev->gz_off = sz / ok;

    dev->last_tick = HAL_GetTick();
    return HAL_OK;
}

void MPU6050_Update_Angle(MPU6050_t *dev)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - dev->last_tick) / 1000.0f;
    dev->last_tick = now;

    /* Sudut dari accelerometer */
    float acc_roll  = atan2f(dev->ay, dev->az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-dev->ax, sqrtf(dev->ay * dev->ay + dev->az * dev->az)) * RAD_TO_DEG;

    if (!dev->angle_init) {
        dev->roll  = acc_roll;
        dev->pitch = acc_pitch;
        dev->angle_init = 1;
        return;
    }

    /* Complementary filter: gyro (jangka pendek) + accel (jangka panjang) */
    dev->roll  = COMP_ALPHA * (dev->roll  + dev->gx * dt) + (1.0f - COMP_ALPHA) * acc_roll;
    dev->pitch = COMP_ALPHA * (dev->pitch + dev->gy * dt) + (1.0f - COMP_ALPHA) * acc_pitch;
}
