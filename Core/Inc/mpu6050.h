/*
 * mpu6050.h
 * Driver MPU6050 untuk STM32 (HAL) - via I2C
 * Taruh di: Core/Inc/mpu6050.h
 */
#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Alamat I2C (HAL pakai alamat 8-bit, jadi di-shift 1)
 * AD0 = GND / tidak disambung -> 0x68
 * AD0 = VCC                   -> 0x69 */
#define MPU6050_ADDR            (0x68 << 1)

/* Register */
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHO_AM_I      0x75

/* Skala (sesuai konfigurasi di MPU6050_Init: ±2g dan ±250 dps) */
#define MPU6050_ACCEL_SCALE     16384.0f   /* LSB per g     */
#define MPU6050_GYRO_SCALE      131.0f     /* LSB per deg/s */

#define MPU6050_I2C_TIMEOUT     100        /* ms */

typedef struct {
    /* Data mentah */
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    int16_t temp_raw;

    /* Data terkonversi */
    float ax, ay, az;       /* g     */
    float gx, gy, gz;       /* deg/s */
    float temp;             /* °C    */

    /* Sudut hasil complementary filter */
    float roll, pitch;      /* derajat */

    /* Offset kalibrasi gyro (deg/s) */
    float gx_off, gy_off, gz_off;

    uint8_t  who_am_i;
    uint32_t last_tick;
    uint8_t  angle_init;

    /* Diagnostik init: langkah terakhir yang dijalankan & status HAL-nya */
    uint8_t           init_step;
    HAL_StatusTypeDef last_status;
} MPU6050_t;

/* Langkah di MPU6050_Init (untuk diagnosa) */
#define MPU_STEP_NONE         0
#define MPU_STEP_PROBE        1   /* IsDeviceReady (ada ACK di alamat?)   */
#define MPU_STEP_WHOAMI_READ  2   /* baca register WHO_AM_I               */
#define MPU_STEP_WHOAMI_VALUE 3   /* nilai WHO_AM_I tidak dikenali        */
#define MPU_STEP_RESET        4
#define MPU_STEP_WAKE         5
#define MPU_STEP_DLPF         6
#define MPU_STEP_SMPLRT       7
#define MPU_STEP_GYRO_CFG     8
#define MPU_STEP_ACCEL_CFG    9

/* Inisialisasi sensor. Return HAL_OK kalau sensor terdeteksi & terkonfigurasi */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

/* Baca accel + temp + gyro sekaligus (14 byte) lalu konversi ke satuan fisik */
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *dev);

/* Kalibrasi offset gyro. Sensor HARUS DIAM selama proses ini */
HAL_StatusTypeDef MPU6050_Calibrate_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_t *dev, uint16_t samples);

/* Hitung roll & pitch (complementary filter). Panggil setelah MPU6050_Read_All */
void MPU6050_Update_Angle(MPU6050_t *dev);

#endif /* MPU6050_H */
