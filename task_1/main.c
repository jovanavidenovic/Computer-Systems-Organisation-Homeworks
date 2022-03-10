/* USER CODE BEGIN Header */
/**
    ******************************************************************************
    * @file                     : main.c
    * @brief                    : Main program body
    ******************************************************************************
    * @attention
    *
    * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
    * All rights reserved.</center></h2>
    *
    * This software component is licensed by ST under BSD 3-Clause license,
    * the "License"; You may not use this file except in compliance with the
    * License. You may obtain a copy of the License at:
    *                                                opensource.org/licenses/BSD-3-Clause
    *
    ******************************************************************************
    */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

/* USER CODE BEGIN PTD */

// a struct that defines a GPIO device (maps GPIOs memory structure)
typedef struct
{
    uint32_t moder; // TODO
    uint32_t otyper; // TODO
    uint32_t ospeedr; // TODO
    uint32_t pupdr;// TODO
    uint32_t idr; // TODO
    uint32_t odr; // TODO
    uint16_t bsr; // TODO
    uint16_t brr; // TODO
} GPIO_device;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

// constants for storing addresses of GPIOs
#define GPIOAd ((GPIO_device*) 0x40020000)  // TODO
#define GPIOBd ((GPIO_device*) 0x40020400)  // TODO
#define GPIOCd ((GPIO_device*) 0x40020800)  // TODO
#define GPIODd ((GPIO_device*) 0x40020C00)	// TODO
#define GPIOEd ((GPIO_device*) 0x40021000)  // TODO
#define GPIOFd ((GPIO_device*) 0x40021400)  // TODO
#define GPIOGd ((GPIO_device*) 0x40021800)  // TODO
#define GPIOHd ((GPIO_device*) 0x40021C00)  // TODO
#define GPIOId ((GPIO_device*) 0x40022000)	// TODO

// MODE constants
#define IN 00		// TODO
#define OUT 01	// TODO
#define AF 10		// TODO
#define ANALOG 11	// TODO

// PUPD constants
#define NO_PULL 00 	// TODO
#define PULL_UP 01 	// TODO
#define PULL_DOWN 10 	// TODO

// OTYPE constants
#define PUSH_PULL 0 // TODO
#define OPEN_DRAIN 1 // TODO

// OSPEED constants
#define S2MHz 00		// TODO
#define S25MHz 01		// TODO
#define S50MHz 10		// TODO
#define S100MHz 11	// TODO

#define RCC_AHB1ENR ((uint32_t*) 0x40023830)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void reset_bit(uint32_t* address, uint8_t p);
void set_bit(uint32_t* address, uint8_t p);
void set16_bit(uint16_t* address, uint8_t p);
void set_two_bits_to(uint32_t* address, uint8_t p, uint8_t n);

void clock_on(GPIO_device * GPIO_addr);
void init_GPIO(GPIO_device * GPIO_addr, uint32_t Pin, uint32_t Mode, uint32_t PUPD, uint32_t OType, uint32_t OSpeed);
void GPIO_pin_write(GPIO_device * GPIO_addr, uint32_t Pin, uint32_t val);
uint32_t GPIO_pin_read(GPIO_device * GPIO_addr, uint32_t Pin);
void delay(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
    * @brief    The application entry point.
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
    /* USER CODE BEGIN 2 */

    // turn on button's GPIO (button is on PA0)
    clock_on(GPIOAd);

    // turn on led's GPIO (leds are on PD12, PD13, PD14, PD15)
    clock_on(GPIODd);

    // init button
    // TODO
    init_GPIO(GPIOAd, 0, IN, NO_PULL, PUSH_PULL, S2MHz);

    // init leds
    // TODO
    init_GPIO(GPIODd, 12, OUT, NO_PULL, PUSH_PULL, S2MHz);
    init_GPIO(GPIODd, 13, OUT, NO_PULL, PUSH_PULL, S2MHz);
    init_GPIO(GPIODd, 14, OUT, NO_PULL, PUSH_PULL, S2MHz);
    init_GPIO(GPIODd, 15, OUT, NO_PULL, PUSH_PULL, S2MHz);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // read button and trigger led sequence

        if(GPIO_pin_read(GPIOAd, 0) == 1){
        	for(uint32_t i = 12; i <= 15; i++){
        		GPIO_pin_write(GPIODd, i, 1);
        		delay();
        	}

        	for(uint32_t i = 12; i <= 15; i++){
        	    GPIO_pin_write(GPIODd, i, 0);
        	    delay();
        	}
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
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
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

void reset_bit(uint32_t* x, uint8_t p){
    *x = *x &(~(1 << p));
}

void set_bit(uint32_t* x, uint8_t p){
    *x = *x |(1 << p);
}

void set16_bit(uint16_t* x, uint8_t p){
    *x = *x |(1 << p);
}


void set_two_bits_to (uint32_t* x, uint8_t p, uint8_t n){
    int i1, i2;
    i2 = n % 2;
    i1 = n / 2;

    if(i2 == 0){
        reset_bit(x, p);
    } else set_bit(x, p);

    if(i1 == 0){
        reset_bit(x, p + 1);
    } else set_bit(x, p + 1);
}


// function for turning on a particular GPIO (enables the port clock)
void clock_on(GPIO_device* GPIO_addr)
{
    // TODO
	uint32_t redniBroj = ((uint32_t)GPIO_addr - (uint32_t)GPIOAd)/0x400;
	set_bit(RCC_AHB1ENR, redniBroj);
}

// function for initializing GPIOs
void init_GPIO(GPIO_device* GPIO_addr, uint32_t Pin, uint32_t Mode, uint32_t PUPD, uint32_t OType, uint32_t OSpeed)
{
    // TODO
	set_two_bits_to(&(GPIO_addr->moder), 2 * Pin, Mode);
	set_two_bits_to(&(GPIO_addr->pupdr),  2  * Pin, PUPD);
	set_two_bits_to(&(GPIO_addr->ospeedr), 2 * Pin, OSpeed);
	if(OType == 1)
		set_bit(&(GPIO_addr->otyper), Pin);
	else reset_bit(&(GPIO_addr->otyper), Pin);
}

// function for setting the value of an output GPIO pin
void GPIO_pin_write(GPIO_device * GPIO_addr, uint32_t Pin, uint32_t val)
{
    // TODO
	//init_GPIO(GPIO_addr, Pin, OUT, NO_PULL, PUSH_PULL, S2MHz);
	if(val == 0)
		set16_bit(&(GPIO_addr->brr), Pin);
	else set16_bit(&(GPIO_addr->bsr), Pin);
}

// function for reading the value of an input GPIO pin
uint32_t GPIO_pin_read(GPIO_device* GPIO_addr, uint32_t Pin)
{
    // TODO
    uint32_t res = (GPIO_addr->idr & (1 << Pin)) == 0;
    if(res == 1)
    	return 0;
    else return 1;
}

// hardcoded delay
void delay(void) {
    volatile int d = 500000;
    while(d--);
}

/* USER CODE END 4 */

/**
    * @brief    This function is executed in case of error occurrence.
    * @retval None
    */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef    USE_FULL_ASSERT
/**
    * @brief    Reports the name of the source file and the source line number
    *                 where the assert_param error has occurred.
    * @param    file: pointer to the source file name
    * @param    line: assert_param error line source number
    * @retval None
    */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
         tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
