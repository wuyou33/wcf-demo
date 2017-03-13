/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/main.c
 * @author  MCD Application Team
 * @version V1.8.0
 * @date    04-November-2016
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw_conf.h"
#include "stdio.h"
#include "wahba_rot.h"

#define CONTROL_LOOP_FREQ 200
/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/



__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;
extern volatile u8 UsbRxBuf[APP_RX_DATA_SIZE];
extern volatile u8 UsbRxLen;

wahba_rotStruct whb;
real dt = 1.0/CONTROL_LOOP_FREQ;

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{

	GenTimInit();
	USBD_Init(&USB_OTG_dev,  USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);
	LedInit();

	delay_us(2000000);
	imu_init();


	wahba_StructInit(&whb,dt);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_SetPriority(SysTick_IRQn, 4);
	SysTick_Config(SystemCoreClock/CONTROL_LOOP_FREQ/8);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

	while (1)
	{
	}
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
	uint8_t str[2048];
	uint32_t len,i;
	static uint32_t cnt=0,t=0, state=0;

	t = toc;

	// Read sensor,no calibration used yet
	lsm9ds0_read();


	Led1(1);
	wahba_rot(lsm9ds0.acc,lsm9ds0.gyr,lsm9ds0.mag,&whb);
	Led1(0);

	len = 0;
	len += sprintf(&str[len],"% d",t);
//	len += sprintf(&str[len],",% 8.3f,% 8.3f,% 8.3f",lsm9ds0.acc[0],lsm9ds0.acc[1],lsm9ds0.acc[2]);
//	len += sprintf(&str[len],",% 8.3f,% 8.3f,% 8.3f",lsm9ds0.gyr[0],lsm9ds0.gyr[1],lsm9ds0.gyr[2]);
//	len += sprintf(&str[len],",% 8.3f,% 8.3f,% 8.3f",lsm9ds0.mag[0],lsm9ds0.mag[1],lsm9ds0.mag[2]);
//	len += sprintf(&str[len],",% 8.3f,% 8.3f,% 8.3f",whb.W[0],whb.W[1],whb.W[2]);
	len += sprintf(&str[len],",% 8.6f,% 8.6f,% 8.6f",whb.Euler[0]*180/M_PI,whb.Euler[1]*180/M_PI,whb.Euler[2]*180/M_PI);
//	len += sprintf(&str[len],",% 8.6f",lsm9ds0.temp);
	len += sprintf(&str[len],"\n");


	VCP_DataTx((u8 *)str,len);
}





#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
