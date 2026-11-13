/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : MPU6050 -> MAVLink v2 lewat USB CDC - Blue Pill STM32F103C8T6
  *                   HSE 8 MHz -> SYSCLK 72 MHz, USB 48 MHz
  *                   I2C1: PB6 = SCL, PB7 = SDA
  *
  * Pesan MAVLink yang dikirim (sysid=1, compid=1):
  *   HEARTBEAT   1 Hz
  *   SCALED_IMU  20 Hz  (acc [mG], gyro [mrad/s], suhu [cdegC])
  *   ATTITUDE    20 Hz  (roll/pitch [rad], yaw = 0)
  *   STATUSTEXT  saat start / error (diagnostik MPU6050)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "usbd_cdc_if.h"
#include "../../c_library_v2/common/mavlink.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

MPU6050_t mpu;

#define MAV_SYS_ID          1U
#define MAV_COMP_ID         MAV_COMP_ID_AUTOPILOT1
#define MAV_HEARTBEAT_MS    1000U
#define MAV_IMU_PERIOD_MS   50U         /* 20 Hz */

/* Buffer TX harus static/global karena USB CDC mengirim secara asynchronous. */
static uint8_t usb_tx_buffer[256];
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t  USB_TxReady(void);
static uint8_t  USB_WaitTxReady(uint32_t timeout_ms);
static void     USB_WaitConfigured(uint32_t timeout_ms);
static uint16_t MAV_Append(uint16_t used, const mavlink_message_t *msg);
static uint8_t  MAV_SendTelemetry(uint8_t with_heartbeat, uint8_t with_imu);
static void     MAV_StatusText(uint8_t severity, const char *text);
static void     I2C_BusRecover(void);
static void     I2C_Scan(void);
static void     MPU_PrintDiag(HAL_StatusTypeDef st);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#define DEG2RAD   0.01745329252f

/* ---------------- USB helper ---------------- */

/* Return 1 jika USB sudah CONFIGURED dan transmit sebelumnya selesai. */
static uint8_t USB_TxReady(void)
{
    USBD_CDC_HandleTypeDef *hcdc;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return 0U;
    }

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL)
    {
        return 0U;
    }

    return (hcdc->TxState == 0U) ? 1U : 0U;
}

static uint8_t USB_WaitTxReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!USB_TxReady())
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            return 0U;
        }
    }
    return 1U;
}

static void USB_WaitConfigured(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
        {
            break;
        }
    }
}

/* ---------------- MAVLink ---------------- */
/* MAV BEGIN */

/* Float -> int16 dengan pembulatan. */
static int16_t MAV_ToI16(float v)
{
    return (int16_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* Serialisasi 'msg' ke usb_tx_buffer mulai dari offset 'used'.
   Return offset baru (tidak berubah jika buffer tidak cukup). */
static uint16_t MAV_Append(uint16_t used, const mavlink_message_t *msg)
{
    uint16_t need = (uint16_t)(MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len);

    if ((uint32_t)used + need > sizeof(usb_tx_buffer))
    {
        return used;
    }
    return (uint16_t)(used + mavlink_msg_to_send_buffer(&usb_tx_buffer[used], msg));
}

/* Kirim HEARTBEAT dan/atau SCALED_IMU + ATTITUDE dalam satu transfer USB.
   Return 1 jika terkirim, 0 jika USB belum siap (coba lagi nanti). */
static uint8_t MAV_SendTelemetry(uint8_t with_heartbeat, uint8_t with_imu)
{
    mavlink_message_t msg;
    uint16_t len = 0U;
    uint32_t now_ms;

    if (!USB_TxReady())
    {
        return 0U;
    }

    now_ms = HAL_GetTick();

    if (with_heartbeat)
    {
        mavlink_msg_heartbeat_pack(MAV_SYS_ID, MAV_COMP_ID, &msg,
                                   MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
                                   0U, 0U, MAV_STATE_ACTIVE);
        len = MAV_Append(len, &msg);
    }

    if (with_imu)
    {
        /* g -> mG, dps -> mrad/s, degC -> cdegC. Magnetometer tidak ada -> 0. */
        mavlink_msg_scaled_imu_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, now_ms,
                                    MAV_ToI16(mpu.ax * 1000.0f),
                                    MAV_ToI16(mpu.ay * 1000.0f),
                                    MAV_ToI16(mpu.az * 1000.0f),
                                    MAV_ToI16(mpu.gx * DEG2RAD * 1000.0f),
                                    MAV_ToI16(mpu.gy * DEG2RAD * 1000.0f),
                                    MAV_ToI16(mpu.gz * DEG2RAD * 1000.0f),
                                    0, 0, 0,
                                    MAV_ToI16(mpu.temp * 100.0f));
        len = MAV_Append(len, &msg);

        /* yaw tidak tersedia (tanpa magnetometer) -> 0 */
        mavlink_msg_attitude_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, now_ms,
                                  mpu.roll  * DEG2RAD,
                                  mpu.pitch * DEG2RAD,
                                  0.0f,
                                  mpu.gx * DEG2RAD,
                                  mpu.gy * DEG2RAD,
                                  mpu.gz * DEG2RAD);
        len = MAV_Append(len, &msg);
    }

    if (len == 0U)
    {
        return 1U;
    }
    return (CDC_Transmit_FS(usb_tx_buffer, len) == USBD_OK) ? 1U : 0U;
}

/* Kirim pesan teks (maks 49 karakter) sebagai STATUSTEXT. */
static void MAV_StatusText(uint8_t severity, const char *text)
{
    mavlink_message_t msg;
    char buf[50];                   /* STATUSTEXT.text = tepat 50 byte */
    uint16_t len;

    memset(buf, 0, sizeof(buf));
    strncpy(buf, text, sizeof(buf) - 1U);

    if (!USB_WaitTxReady(200U))
    {
        return;                     /* USB belum siap / host belum membuka port */
    }

    mavlink_msg_statustext_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, severity, buf, 0U, 0U);
    len = MAV_Append(0U, &msg);
    (void)CDC_Transmit_FS(usb_tx_buffer, len);
}

/* MAV END */

/* ---------------- Diagnostik I2C / MPU6050 ---------------- */

/* Lepas bus I2C yang nyangkut (slave menahan SDA LOW / flag BUSY):
   kirim 9 clock manual di SCL lalu STOP, kemudian init ulang I2C. */
static void I2C_BusRecover(void)
{
    GPIO_InitTypeDef g = {0};
    int i;

    (void)HAL_I2C_DeInit(&hi2c1);

    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;          /* PB6 = SCL, PB7 = SDA */
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    for (i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* Kondisi STOP: SDA naik saat SCL HIGH */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    MX_I2C1_Init();                              /* pin kembali ke AF I2C */
}

/* Scan semua alamat 7-bit, kirim yang menjawab (ACK) sebagai STATUSTEXT. */
static void I2C_Scan(void)
{
    char line[50];
    size_t pos;
    uint8_t found = 0U;
    uint8_t addr;

    pos = (size_t)snprintf(line, sizeof(line), "I2C scan:");

    for (addr = 1U; addr < 127U; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1U, 5U) == HAL_OK)
        {
            if (pos + 6U < sizeof(line))
            {
                pos += (size_t)snprintf(line + pos, sizeof(line) - pos, " 0x%02X", addr);
            }
            found++;
        }
    }

    if (found == 0U)
    {
        snprintf(line + pos, sizeof(line) - pos, " tidak ada device");
    }
    MAV_StatusText(MAV_SEVERITY_WARNING, line);
}

/* Kirim langkah init yang gagal + petunjuk sebagai STATUSTEXT. */
static void MPU_PrintDiag(HAL_StatusTypeDef st)
{
    static const char *const step_name[] = {
        "-", "PROBE", "WHOAMI_RD", "WHOAMI_VAL",
        "RESET", "WAKE", "DLPF", "SMPLRT", "GYRO_CFG", "ACCEL_CFG"
    };
    static const char *const st_name[] = { "OK", "ERROR", "BUSY", "TIMEOUT" };

    char msg[50];
    uint32_t err = HAL_I2C_GetError(&hi2c1);
    uint8_t step = mpu.init_step;

    if (step >= (sizeof(step_name) / sizeof(step_name[0]))) step = 0U;

    snprintf(msg, sizeof(msg), "MPU ERR %s hal=%s i2c=0x%02lX id=0x%02X",
             step_name[step],
             ((unsigned)st < 4U) ? st_name[st] : "?",
             (unsigned long)err,
             (unsigned)mpu.who_am_i);
    MAV_StatusText(MAV_SEVERITY_ERROR, msg);

    if (st == HAL_BUSY)
    {
        MAV_StatusText(MAV_SEVERITY_ERROR, "Bus BUSY: SDA/SCL LOW (short/tanpa pull-up)");
    }
    else if (err & HAL_I2C_ERROR_AF)
    {
        MAV_StatusText(MAV_SEVERITY_ERROR, "NACK: cek VCC/GND, SDA=PB7, SCL=PB6, AD0");
    }
    else if (err & HAL_I2C_ERROR_BERR)
    {
        MAV_StatusText(MAV_SEVERITY_ERROR, "Bus error: kabel longgar / SDA-SCL tertukar");
    }
    else if (err & HAL_I2C_ERROR_ARLO)
    {
        MAV_StatusText(MAV_SEVERITY_ERROR, "Arbitration lost: noise / short di bus");
    }
    else
    {
        MAV_StatusText(MAV_SEVERITY_ERROR, "Timeout: cek pull-up 4.7k & tegangan 3.3V");
    }
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t last_hb  = 0U;
  uint32_t last_imu = 0U;
  uint8_t  hb_due   = 1U;     /* kirim heartbeat pertama segera */
  /* USER CODE END 1 */

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();

  /* USER CODE BEGIN SysInit */
  /* Paksa host mendeteksi ulang USB setiap reset/flash:
     tarik PA12 (USB D+) ke LOW sebentar sebelum USB diinisialisasi. */
  {
      GPIO_InitTypeDef g = {0};
      g.Pin   = GPIO_PIN_12;
      g.Mode  = GPIO_MODE_OUTPUT_PP;
      g.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &g);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_Delay(50);
  }
  /* USER CODE END SysInit */

  MX_I2C1_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  USB_WaitConfigured(5000U);
  HAL_Delay(500);             /* beri waktu aplikasi di PC membuka port */

  /* Pastikan bus I2C bersih sebelum mulai */
  I2C_BusRecover();

  /* Inisialisasi sensor; ulangi sampai berhasil, laporkan error via STATUSTEXT */
  {
      HAL_StatusTypeDef init_st;
      char info[50];

      while ((init_st = MPU6050_Init(&hi2c1, &mpu)) != HAL_OK)
      {
          (void)MAV_SendTelemetry(1U, 0U);    /* tetap kirim heartbeat */
          MPU_PrintDiag(init_st);
          I2C_Scan();

          if (init_st == HAL_BUSY)
          {
              I2C_BusRecover();
          }
          HAL_Delay(1000);
      }

      snprintf(info, sizeof(info), "MPU OK, WHO_AM_I=0x%02X%s",
               (unsigned)mpu.who_am_i,
               (mpu.who_am_i == 0x68) ? "" : " (clone)");
      MAV_StatusText(MAV_SEVERITY_INFO, info);
  }

  MAV_StatusText(MAV_SEVERITY_NOTICE, "Kalibrasi gyro: jangan gerakkan sensor");
  (void)MPU6050_Calibrate_Gyro(&hi2c1, &mpu, 400);
  MAV_StatusText(MAV_SEVERITY_INFO, "Kalibrasi selesai");

  last_hb  = HAL_GetTick();
  last_imu = last_hb;
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    if (MPU6050_Read_All(&hi2c1, &mpu) == HAL_OK)
    {
        MPU6050_Update_Angle(&mpu);
    }

    if ((now - last_hb) >= MAV_HEARTBEAT_MS)
    {
        hb_due = 1U;
    }

    if (hb_due || ((now - last_imu) >= MAV_IMU_PERIOD_MS))
    {
        uint8_t imu_due = ((now - last_imu) >= MAV_IMU_PERIOD_MS) ? 1U : 0U;

        if (MAV_SendTelemetry(hb_due, imu_due))
        {
            if (hb_due)  { last_hb  = now; hb_due = 0U; }
            if (imu_due) { last_imu = now; }
        }
    }

    HAL_Delay(5);             /* ~200 Hz baca sensor */
    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  *        HSE 8 MHz -> PLL x9 = 72 MHz, USB = 72 / 1.5 = 48 MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* PCLK1 = 36 MHz (maks) */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization (pin PB6/PB7 diatur di HAL_I2C_MspInit
  *        pada stm32f1xx_hal_msp.c, dibuat otomatis oleh CubeMX)
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();   /* pin HSE (PD0/PD1) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
