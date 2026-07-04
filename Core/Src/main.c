/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t podaci_spremni  = 0;
volatile uint8_t uzorak_spreman  = 0;
uint32_t ir_buffer[2000];
uint32_t red_buffer[2000];
uint16_t buffer_index  = 0;
uint8_t  buffer_pun    = 0;
uint32_t zadnji_uzorak = 0;

//dodatna varijabla za bolji bpm
static uint8_t prst_bio_prisutan = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX30100_ADDR        0xAE  // 0x57 << 1
#define MAX30100_REG_MODE    0x06
#define MAX30100_REG_SPO2    0x07
#define MAX30100_REG_LED     0x09
#define MAX30100_REG_FIFO    0x05
#define MAX30100_REG_INT_EN  0x01
#define MAX_UZORKOVANJE      2000

void MAX30100_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    // Reset
    data = 0x40;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, MAX30100_REG_MODE, 1, &data, 1, 100);
    HAL_Delay(100);

    // SpO2 mode
    data = 0x03;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, MAX30100_REG_MODE, 1, &data, 1, 100);

    // SpO2 config: 100Hz, 1600us pulse
    data = 0x47;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, MAX30100_REG_SPO2, 1, &data, 1, 100);

    // LED struja: IR 36mA + RED 36mA (0xFF je previse, sensor satura)
    data = 0xAA;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, MAX30100_REG_LED, 1, &data, 1, 100);

    // ocisti FIFO pokazivace
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, 0x02, 1, &data, 1, 100);
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, 0x03, 1, &data, 1, 100);
    HAL_I2C_Mem_Write(hi2c, MAX30100_ADDR, 0x04, 1, &data, 1, 100);
}

void MAX30100_ReadFifo(I2C_HandleTypeDef *hi2c, uint32_t *ir, uint32_t *red)
{
    uint8_t buf[4];
    HAL_I2C_Mem_Read(hi2c, MAX30100_ADDR, MAX30100_REG_FIFO, 1, buf, 4, 100);
    *ir  = ((uint32_t)buf[0] << 8) | buf[1];
    *red = ((uint32_t)buf[2] << 8) | buf[3];
}

// port oxullo BeatDetector algoritma za STM32 HAL
#define BEATDETECTOR_INIT_HOLDOFF          2000
#define BEATDETECTOR_CANDIDACY_TRESHOLD    0.02f
#define BEATDETECTOR_MASKING_HOLDOFF       500
#define BEATDETECTOR_BPFILTER_ALPHA        0.6f
#define BEATDETECTOR_MIN_THRESHOLD         0.20f
#define BEATDETECTOR_MAX_THRESHOLD         13.0f
#define BEATDETECTOR_STEP_RESISTRY         0.0005f
#define BEATDETECTOR_STEP_DECAY            0.002f

typedef enum {
    BEATDETECTOR_STATE_INIT,
    BEATDETECTOR_STATE_WAITING,
    BEATDETECTOR_STATE_FOLLOWING_SLOPE,
    BEATDETECTOR_STATE_MAYBE_DETECTED,
    BEATDETECTOR_STATE_MASKING
} BeatDetectorState;

static BeatDetectorState  beat_state     = BEATDETECTOR_STATE_INIT;
static float              beat_threshold = BEATDETECTOR_MAX_THRESHOLD;
static float              beat_bpm       = 0.0f;
static uint32_t           beat_ts_last   = 0;
static uint32_t           beat_ts_first  = 0;
static uint32_t           beat_count     = 0;

uint8_t BeatDetector_Update(float sample)
{
    uint8_t beat_detected = 0;
    uint32_t now = HAL_GetTick();

    switch (beat_state)
    {
        case BEATDETECTOR_STATE_INIT:
            if (now > BEATDETECTOR_INIT_HOLDOFF) {
                beat_state = BEATDETECTOR_STATE_WAITING;
            }
            break;

        case BEATDETECTOR_STATE_WAITING:
            if (sample > beat_threshold) {
                beat_state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }
            if (sample > BEATDETECTOR_MIN_THRESHOLD) {
                beat_threshold -= BEATDETECTOR_STEP_DECAY;
            } else {
                beat_threshold += BEATDETECTOR_STEP_RESISTRY;
            }
            if (beat_threshold < BEATDETECTOR_MIN_THRESHOLD)
                beat_threshold = BEATDETECTOR_MIN_THRESHOLD;
            if (beat_threshold > BEATDETECTOR_MAX_THRESHOLD)
                beat_threshold = BEATDETECTOR_MAX_THRESHOLD;
            break;

        case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
            if (sample < beat_threshold) {
                beat_state = BEATDETECTOR_STATE_MAYBE_DETECTED;
            }
            beat_threshold = sample * BEATDETECTOR_CANDIDACY_TRESHOLD;
            if (beat_threshold < BEATDETECTOR_MIN_THRESHOLD)
                beat_threshold = BEATDETECTOR_MIN_THRESHOLD;
            break;

        case BEATDETECTOR_STATE_MAYBE_DETECTED:
            if (sample + BEATDETECTOR_CANDIDACY_TRESHOLD < beat_threshold) {
                beat_detected = 1;
                uint32_t interval = now - beat_ts_last;
                if (beat_ts_last > 0 && interval > 300 && interval < 2000) {
                    beat_bpm = 60000.0f / (float)interval;
                }
                beat_ts_last = now;
                beat_count++;
                beat_state = BEATDETECTOR_STATE_MASKING;
            } else {
                beat_state = BEATDETECTOR_STATE_WAITING;
            }
            break;

        case BEATDETECTOR_STATE_MASKING:
            if (now - beat_ts_last > BEATDETECTOR_MASKING_HOLDOFF) {
                beat_state = BEATDETECTOR_STATE_WAITING;
            }
            break;
    }

    return beat_detected;
}

float BeatDetector_GetBPM(void)
{
    return beat_bpm;
}

int32_t Calculate_SpO2(uint32_t *ir_buf, uint32_t *red_buf, uint16_t size)
{
    if (size > MAX_UZORKOVANJE) size = MAX_UZORKOVANJE;

    #define NUM_WINDOWS 9
    float ratios[NUM_WINDOWS];
    uint16_t window_size = size / NUM_WINDOWS;

    for (int w = 0; w < NUM_WINDOWS; w++)
    {
        uint16_t start = w * window_size;
        uint16_t end   = start + window_size;

        uint32_t ir_max = 0, ir_min = 0xFFFFFFFF;
        uint32_t red_max = 0, red_min = 0xFFFFFFFF;
        uint64_t ir_sum = 0, red_sum = 0;

        for (int i = start; i < end; i++) {
            ir_sum  += ir_buf[i];
            red_sum += red_buf[i];
            if (ir_buf[i]  > ir_max)  ir_max  = ir_buf[i];
            if (ir_buf[i]  < ir_min)  ir_min  = ir_buf[i];
            if (red_buf[i] > red_max) red_max = red_buf[i];
            if (red_buf[i] < red_min) red_min = red_buf[i];
        }

        float ir_ac  = (float)(ir_max - ir_min);
        float red_ac = (float)(red_max - red_min);
        float ir_dc  = (float)(ir_sum  / window_size);
        float red_dc = (float)(red_sum / window_size);

        if (ir_ac < 1.0f || ir_dc < 1.0f)
            ratios[w] = 0.0f;
        else
            ratios[w] = (red_ac / red_dc) / (ir_ac / ir_dc);
    }

    for (int i = 0; i < NUM_WINDOWS - 1; i++)
        for (int j = i + 1; j < NUM_WINDOWS; j++)
            if (ratios[i] > ratios[j]) {
                float tmp = ratios[i];
                ratios[i] = ratios[j];
                ratios[j] = tmp;
            }

    float ratio = ratios[NUM_WINDOWS / 2];

    if (ratio < 0.1f || ratio > 1.3f ) return 0;

    // kalibrirana formula za MAX30100
    int32_t spo2 = (int32_t)(109.0f - 14.0f * ratio);

    if (spo2 > 99) spo2 = 99;
    if (spo2 < 80)  spo2 = 80;

    return spo2;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    char     buffer[32];
    uint32_t zadnje_vrijeme = 0;
    int32_t  bpm  = 0;
    int32_t  spo2 = 0;

    HAL_TIM_Base_Start_IT(&htim2);

    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("Pokretanje...", Font_11x18, White);
    ssd1306_UpdateScreen();
    HAL_Delay(500);

    MAX30100_Init(&hi2c1);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 10);
    ssd1306_WriteString("Spreman!", Font_11x18, White);
    ssd1306_UpdateScreen();
    HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    	if (uzorak_spreman)
    	{
    	    uzorak_spreman = 0;

    	    uint32_t ir, red;
    	    MAX30100_ReadFifo(&hi2c1, &ir, &red);

    	    static float ir_dc = 0.0f;
    	    ir_dc = ir_dc * 0.99f + (float)ir * 0.01f;
    	    float ir_ac = (float)ir - ir_dc;

    	    // uklanjanje visokofrekventnog suma na AC signalu
    	    static float ir_ac_filtered = 0.0f;
    	    ir_ac_filtered = ir_ac_filtered * 0.75f + ir_ac * 0.25f;

    	    BeatDetector_Update(ir_ac_filtered);  // ← filtrirani signal

    	    ir_buffer[buffer_index]  = ir;
    	    red_buffer[buffer_index] = red;
    	    buffer_index++;
    	    if (buffer_index >= 2000) {
    	        buffer_index = 0;
    	        buffer_pun   = 1;
    	    }
    	}

        // prikaz svakih 1000ms, samo kad je buffer pun
        if (buffer_pun && (HAL_GetTick() - zadnje_vrijeme >= 1000))
        {
            zadnje_vrijeme = HAL_GetTick();

            // provjera je li prst prisutan (zadnjih 10 uzoraka)
            uint8_t prst_prisutan = 1;
            for (int i = 0; i < 10; i++) {
                uint16_t idx = (buffer_index + 2000 - 1 - i) % 2000;
                if (ir_buffer[idx] < 13500) {
                    prst_prisutan = 0;
                    break;
                }
            }

            ssd1306_Fill(Black);

            if (prst_prisutan)
               {
                float novi_bpm_f = BeatDetector_GetBPM();
                int32_t novi_bpm = (int32_t)novi_bpm_f;
                int32_t novi_spo2 = Calculate_SpO2(ir_buffer, red_buffer, 2000);

                static int32_t bpm_history[7] = {0, 0, 0, 0, 0, 0, 0};
                static uint8_t bpm_idx = 0;

                if (!prst_bio_prisutan)
                   {
                    beat_state     = BEATDETECTOR_STATE_INIT;
                    beat_threshold = BEATDETECTOR_MAX_THRESHOLD;
                    beat_bpm       = 0.0f;
                    beat_ts_last   = 0;
                	beat_count     = 0;

                	for (int i = 0; i < 7; i++) bpm_history[i] = 0;
                         bpm = 0;
                    }
                     prst_bio_prisutan = 1;

                if (novi_bpm >= 50 && novi_bpm <= 180)
                {
                    // odbaci ako se previse razlikuje od trenutnog prosjeka
                    if (bpm == 0 || (novi_bpm > bpm - 20 && novi_bpm < bpm + 20))
                    {
                        bpm_history[bpm_idx] = novi_bpm;
                        bpm_idx = (bpm_idx + 1) % 7;
                        int32_t sum = 0;
                        int32_t count = 0;
                        for (int i = 0; i < 7; i++) {
                            if (bpm_history[i] > 0) {
                                sum += bpm_history[i];
                                count++;
                            }
                        }
                        if (count > 0) bpm = sum / count;
                    }
                }

                static int32_t spo2_history[5] = {0, 0, 0, 0, 0};
                static uint8_t spo2_idx = 0;

                if (novi_spo2 >= 85 && novi_spo2 <= 100)
                {
                    spo2_history[spo2_idx] = novi_spo2;
                    spo2_idx = (spo2_idx + 1) % 5;
                    int32_t spo2_sum = 0;
                    for (int i = 0; i < 5; i++) spo2_sum += spo2_history[i];
                    spo2 = spo2_sum / 5;
                }


                if (bpm > 0 && spo2 > 0)
                {
                    sprintf(buffer, "BPM: %ld", bpm);
                    ssd1306_SetCursor(0, 0);
                    ssd1306_WriteString(buffer, Font_11x18, White);


                    sprintf(buffer, "SpO2:%ld%%", spo2);
                    ssd1306_SetCursor(0, 25);
                    ssd1306_WriteString(buffer, Font_11x18, White);

                }
                else
                {
                    ssd1306_SetCursor(0, 10);
                    ssd1306_WriteString("Miruj...", Font_11x18, White);
                }
            }
            else
            {
            	prst_bio_prisutan = 0;
                bpm  = 0;
                spo2 = 0;
                ssd1306_SetCursor(0, 20);
                ssd1306_WriteString("Stavi prst", Font_11x18, White);
            }

            ssd1306_UpdateScreen();
        }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        uzorak_spreman = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
        podaci_spremni = 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
