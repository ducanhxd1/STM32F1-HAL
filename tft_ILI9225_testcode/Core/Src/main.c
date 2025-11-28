/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Change the width and height if required (defined in portrait mode)
// or use the constructor to over-ride defaults
#define TFT_WIDTH  176
#define TFT_HEIGHT 220
#define TFT_SIZE (TFT_WIDTH * TFT_HEIGHT)

// Generic commands used by TFT_eSPI.cpp
#define TFT_NOP     0x00
#define TFT_SWRST   0x28

#define TFT_CASET 0
#define TFT_PASET 0

#define TFT_CASET1     ILI9225_HORIZONTAL_WINDOW_ADDR2
#define TFT_CASET2     ILI9225_HORIZONTAL_WINDOW_ADDR1

#define TFT_PASET1     ILI9225_VERTICAL_WINDOW_ADDR2
#define TFT_PASET2     ILI9225_VERTICAL_WINDOW_ADDR1

#define TFT_RAM_ADDR1  ILI9225_RAM_ADDR_SET1
#define TFT_RAM_ADDR2  ILI9225_RAM_ADDR_SET2

#define TFT_RAMWR      ILI9225_GRAM_DATA_REG

#define TFT_MAD_BGR 0x10
#define TFT_MAD_RGB 0x00

#ifdef TFT_RGB_ORDER
  #if (TFT_RGB_ORDER == 1)
    #define TFT_MAD_COLOR_ORDER TFT_MAD_RGB
  #else
    #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
  #endif
#else
  #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
#endif

/* ILI9225 Registers */
#define ILI9225_DRIVER_OUTPUT_CTRL      0x01  // Driver Output Control
#define ILI9225_LCD_AC_DRIVING_CTRL     0x02  // LCD AC Driving Control
#define ILI9225_ENTRY_MODE              0x03  // Entry Mode
#define ILI9225_DISP_CTRL1              0x07  // Display Control 1
#define ILI9225_BLANK_PERIOD_CTRL1      0x08  // Blank Period Control
#define ILI9225_FRAME_CYCLE_CTRL        0x0B  // Frame Cycle Control
#define ILI9225_INTERFACE_CTRL          0x0C  // Interface Control
#define ILI9225_OSC_CTRL                0x0F  // Osc Control
#define ILI9225_POWER_CTRL1             0x10  // Power Control 1
#define ILI9225_POWER_CTRL2             0x11  // Power Control 2
#define ILI9225_POWER_CTRL3             0x12  // Power Control 3
#define ILI9225_POWER_CTRL4             0x13  // Power Control 4
#define ILI9225_POWER_CTRL5             0x14  // Power Control 5
#define ILI9225_VCI_RECYCLING           0x15  // VCI Recycling
#define ILI9225_RAM_ADDR_SET1           0x20  // Horizontal GRAM Address Set
#define ILI9225_RAM_ADDR_SET2           0x21  // Vertical GRAM Address Set
#define ILI9225_GRAM_DATA_REG           0x22  // GRAM Data Register
#define ILI9225_GATE_SCAN_CTRL          0x30  // Gate Scan Control Register
#define ILI9225_VERTICAL_SCROLL_CTRL1   0x31  // Vertical Scroll Control 1 Register
#define ILI9225_VERTICAL_SCROLL_CTRL2   0x32  // Vertical Scroll Control 2 Register
#define ILI9225_VERTICAL_SCROLL_CTRL3   0x33  // Vertical Scroll Control 3 Register
#define ILI9225_PARTIAL_DRIVING_POS1    0x34  // Partial Driving Position 1 Register
#define ILI9225_PARTIAL_DRIVING_POS2    0x35  // Partial Driving Position 2 Register
#define ILI9225_HORIZONTAL_WINDOW_ADDR1 0x36  // Horizontal Address Start Position
#define ILI9225_HORIZONTAL_WINDOW_ADDR2 0x37  // Horizontal Address End Position
#define ILI9225_VERTICAL_WINDOW_ADDR1   0x38  // Vertical Address Start Position
#define ILI9225_VERTICAL_WINDOW_ADDR2   0x39  // Vertical Address End Position
#define ILI9225_GAMMA_CTRL1             0x50  // Gamma Control 1
#define ILI9225_GAMMA_CTRL2             0x51  // Gamma Control 2
#define ILI9225_GAMMA_CTRL3             0x52  // Gamma Control 3
#define ILI9225_GAMMA_CTRL4             0x53  // Gamma Control 4
#define ILI9225_GAMMA_CTRL5             0x54  // Gamma Control 5
#define ILI9225_GAMMA_CTRL6             0x55  // Gamma Control 6
#define ILI9225_GAMMA_CTRL7             0x56  // Gamma Control 7
#define ILI9225_GAMMA_CTRL8             0x57  // Gamma Control 8
#define ILI9225_GAMMA_CTRL9             0x58  // Gamma Control 9
#define ILI9225_GAMMA_CTRL10            0x59  // Gamma Control 10

#define LCD_nCS_LOW()	HAL_GPIO_WritePin(GPIOB, TFT_CS_Pin, 0) // ghi chan CS xuong muc 0
#define LCD_nCS_HIGH()	HAL_GPIO_WritePin(GPIOB, TFT_CS_Pin, 1)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void ILI9225_WriteCommand(uint8_t command)
//{
//	LCD_nCS_LOW();
//	HAL_GPIO_WritePin(GPIOB, TFT_CMD_Pin, 0); // RS = 0 => CMD
//	HAL_SPI_Transmit(&hspi1, &command, 1, 1000);
//	LCD_nCS_HIGH();
//}

//void ILI9225_WriteData(uint8_t data)
//{
//	LCD_nCS_LOW();
//	HAL_GPIO_WritePin(GPIOB, TFT_CMD_Pin, 1); // RS = 1 -> DATA
//	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
//	LCD_nCS_HIGH();
//}

void ILI9225_WriteCommand16(uint16_t cmd)
{
    uint8_t cmd_buf[2];
    // ILI9225 yêu cầu gửi Byte Cao (MSB) trước, Byte Thấp (LSB) sau
    cmd_buf[0] = (uint8_t)(cmd >> 8);  // Byte Cao
    cmd_buf[1] = (uint8_t)(cmd & 0xFF); // Byte Thấp

    LCD_nCS_LOW();
    HAL_GPIO_WritePin(GPIOB, TFT_CMD_Pin, 0); // TFT_CMD_Pin là nối với chân RS = 0 -> DATA
    HAL_SPI_Transmit(&hspi1, cmd_buf, 2, 1000); // Gửi 2 bytes
    LCD_nCS_HIGH();
}

void ILI9225_WriteData16(uint16_t data)
{
    uint8_t data_buf[2];
    // ILI9225 yêu cầu gửi Byte Cao (MSB) trước, Byte Thấp (LSB) sau
    data_buf[0] = (uint8_t)(data >> 8);  // Byte Cao
    data_buf[1] = (uint8_t)(data & 0xFF); // Byte Thấp

    LCD_nCS_LOW();
    HAL_GPIO_WritePin(GPIOB, TFT_CMD_Pin, 1); // RS = 1 -> DATA
    HAL_SPI_Transmit(&hspi1, data_buf, 2, 1000); // Gửi 2 bytes
    LCD_nCS_HIGH();
}

void ILI9225_WriteRegister(uint16_t cmd, uint16_t data)
{
	ILI9225_WriteCommand16(cmd);
//	ILI9225_WriteData(data);
	ILI9225_WriteData16(data);
}

void ILI9225_Init()
{
	// 1. SW reset; reset pin PB1
	HAL_GPIO_WritePin(GPIOB, TFT_RST_Pin, 0);
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB, TFT_RST_Pin, 1);
	HAL_Delay(200);

	/* Start Initial Sequence */
	/* Set SS bit and direction output from S528 to S1 */
	ILI9225_WriteRegister(ILI9225_POWER_CTRL1, 0x0000); // Set SAP,DSTB,STB
	ILI9225_WriteRegister(ILI9225_POWER_CTRL2, 0x0000); // Set APON,PON,AON,VCI1EN,VC
	ILI9225_WriteRegister(ILI9225_POWER_CTRL3, 0x0000); // Set BT,DC1,DC2,DC3
	ILI9225_WriteRegister(ILI9225_POWER_CTRL4, 0x0000); // Set GVDD
	ILI9225_WriteRegister(ILI9225_POWER_CTRL5, 0x0000); // Set VCOMH/VCOML voltage
	HAL_Delay(40);

	// Power-on sequence
	ILI9225_WriteRegister(ILI9225_POWER_CTRL2, 0x0018); // Set APON,PON,AON,VCI1EN,VC
	ILI9225_WriteRegister(ILI9225_POWER_CTRL3, 0x6121); // Set BT,DC1,DC2,DC3
	ILI9225_WriteRegister(ILI9225_POWER_CTRL4, 0x006F); // Set GVDD   /*007F 0088 */
	ILI9225_WriteRegister(ILI9225_POWER_CTRL5, 0x495F); // Set VCOMH/VCOML voltage
	ILI9225_WriteRegister(ILI9225_POWER_CTRL1, 0x0800); // Set SAP,DSTB,STB
	HAL_Delay(10);

	ILI9225_WriteRegister(ILI9225_POWER_CTRL2, 0x103B); // Set APON,PON,AON,VCI1EN,VC
	HAL_Delay(50);

	ILI9225_WriteRegister(ILI9225_DRIVER_OUTPUT_CTRL, 0x011C); // set the display line number and display direction
	ILI9225_WriteRegister(ILI9225_LCD_AC_DRIVING_CTRL, 0x0100); // set 1 line inversion
	ILI9225_WriteRegister(ILI9225_ENTRY_MODE, 0x1038); // set GRAM write direction and BGR=1.
	ILI9225_WriteRegister(ILI9225_DISP_CTRL1, 0x0000); // Display off
	ILI9225_WriteRegister(ILI9225_BLANK_PERIOD_CTRL1, 0x0808); // set the back porch and front porch
	ILI9225_WriteRegister(ILI9225_FRAME_CYCLE_CTRL, 0x1100); // set the clocks number per line
	ILI9225_WriteRegister(ILI9225_INTERFACE_CTRL, 0x0000); // CPU interface
	ILI9225_WriteRegister(ILI9225_OSC_CTRL, 0x0D01); // Set Osc  /*0e01*/
	ILI9225_WriteRegister(ILI9225_VCI_RECYCLING, 0x0020); // Set VCI recycling
	ILI9225_WriteRegister(ILI9225_RAM_ADDR_SET1, 0x0000); // RAM Address
	ILI9225_WriteRegister(ILI9225_RAM_ADDR_SET2, 0x0000); // RAM Address

	/* Set GRAM area */
	ILI9225_WriteRegister(ILI9225_GATE_SCAN_CTRL, 0x0000);
	ILI9225_WriteRegister(ILI9225_VERTICAL_SCROLL_CTRL1, 0x00DB);
	ILI9225_WriteRegister(ILI9225_VERTICAL_SCROLL_CTRL2, 0x0000);
	ILI9225_WriteRegister(ILI9225_VERTICAL_SCROLL_CTRL3, 0x0000);
	ILI9225_WriteRegister(ILI9225_PARTIAL_DRIVING_POS1, 0x00DB);
	ILI9225_WriteRegister(ILI9225_PARTIAL_DRIVING_POS2, 0x0000);
	ILI9225_WriteRegister(ILI9225_HORIZONTAL_WINDOW_ADDR1, 0x00AF);
	ILI9225_WriteRegister(ILI9225_HORIZONTAL_WINDOW_ADDR2, 0x0000);
	ILI9225_WriteRegister(ILI9225_VERTICAL_WINDOW_ADDR1, 0x00DB);
	ILI9225_WriteRegister(ILI9225_VERTICAL_WINDOW_ADDR2, 0x0000);

	/* Set GAMMA curve */
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL1, 0x0000);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL2, 0x0808);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL3, 0x080A);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL4, 0x000A);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL5, 0x0A08);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL6, 0x0808);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL7, 0x0000);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL8, 0x0A00);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL9, 0x0710);
	ILI9225_WriteRegister(ILI9225_GAMMA_CTRL10, 0x0710);

	ILI9225_WriteRegister(ILI9225_DISP_CTRL1, 0x0012);
	HAL_Delay(50);

	ILI9225_WriteRegister(ILI9225_DISP_CTRL1, 0x1017);
}

/**
 * @brief Tô đầy màn hình LCD bằng một màu sắc 16-bit cụ thể.
 * @param color: Giá trị màu 16-bit (R5G6B5)
 */
void ILI9225_FillScreen(uint16_t color)
{
    // 1. Đặt lại con trỏ vẽ về (0, 0)
    // Thanh ghi 20h (RAM Address Set 1 - Cột X)
    ILI9225_WriteRegister(0x20, 0x0000);
    // Thanh ghi 21h (RAM Address Set 2 - Hàng Y)
    ILI9225_WriteRegister(0x21, 0x0000);

    // 2. Chọn Lệnh Ghi Dữ liệu vào GRAM (0x22)
    // Lệnh này chỉ cần gửi 8-bit Command (RS=0)
    ILI9225_WriteCommand16(0x22);

    // 3. Gửi Dữ liệu Màu (color) cho toàn bộ màn hình

    // Chuẩn bị 2 byte dữ liệu màu để gửi lặp lại
    uint8_t color_buf[2];
    color_buf[0] = (uint8_t)(color >> 8);  // Byte Cao
    color_buf[1] = (uint8_t)(color & 0xFF); // Byte Thấp

    // Bắt đầu truyền dữ liệu (RS=1)
    LCD_nCS_LOW();
    HAL_GPIO_WritePin(GPIOB, TFT_CMD_Pin, 1); // RS = 1 -> DATA

    // Gửi 38720 pixel (mỗi pixel là 2 bytes)
    for (uint32_t i = 0; i < TFT_SIZE; i++)
    {
        // Sử dụng HAL_SPI_Transmit trực tiếp để tránh overhead của hàm con
        // và giữ CS (Chip Select) LOW trong suốt quá trình vẽ
        HAL_SPI_Transmit(&hspi1, color_buf, 2, 10);
    }

    LCD_nCS_HIGH();
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ILI9225_Init();
  #define COLOR_RED   0xF800
  #define COLOR_GREEN 0x07E0
  #define COLOR_BLUE  0x001F
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // Test màu Đỏ
	  ILI9225_FillScreen(COLOR_RED);
	  HAL_Delay(1000);

	  // Test màu Xanh lá
	  ILI9225_FillScreen(COLOR_GREEN);
	  HAL_Delay(1000);

	  // Test màu Xanh dương
	  ILI9225_FillScreen(COLOR_BLUE);
	  HAL_Delay(1000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_CMD_Pin|TFT_RST_Pin|TFT_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TFT_CMD_Pin TFT_RST_Pin TFT_CS_Pin */
  GPIO_InitStruct.Pin = TFT_CMD_Pin|TFT_RST_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
