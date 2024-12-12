#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <core_cm7.h>
#include "cmsis_os.h"
#include <stdint.h>
//#define GPIO_NUMBER (16U)

//extern TIM_HandleTypeDef htim8;

//#define mor_RearLeft	 								TIM8 -> CCR4// khong co
//#define mor_RearLeft_next							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)
//#define mor_RearLeft_back 						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)

//#define mor_0h	 								TIM8 -> CCR4// khong co
//#define mor_0h_next							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)
//#define mor_0h_back 						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)

#define mor_4h	 								TIM8 -> CCR4// khong co
#define mor_4h_back							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET)
#define mor_4h_next 						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET)

//#define mor_luaBanh	 							TIM8 -> CCR3 // RearLeft --> FrontRight
//#define mor_luaBanh_next						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET)
//#define mor_luaBanh_back 					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET)


#define mor_8h	 								TIM8 -> CCR2	//FrontLeft --> RearRight
#define mor_8h_back							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET)
#define mor_8h_next 						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET)

//#define mor_8hRo	 							TIM8->CCR1 //RearRight --> RearLeft
//#define mor_8hRo_back						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET)
//#define mor_8hRo_next 					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET)

//#define mor_8h	 								TIM8->CCR1 //RearRight --> RearLeft
//#define mor_8h_back							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET)
//#define mor_8h_next 						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET)

#define mor_8hRo	 								TIM8->CCR1 //RearRight --> RearLeft
#define mor_8hRo_next							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET)
#define mor_8hRo_back 						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET)

//#define Pressure                     TIM12 -> CCR2 //PWM pressure

// --------------------------------
#define mor_4hRo								TIM5->CCR1
#define mor_4hRo_back						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET)
#define mor_4hRo_next						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET)

//#define mor_cuonGiua								TIM5->CCR1
//#define mor_cuonGiua_next						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET)
//#define mor_cuonGiua_back						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET)

//#define mor_0hRo								TIM5->CCR1
//#define mor_0hRo_back						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET)
//#define mor_0hRo_next						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET)

//#define mor_luaBanh 									TIM5->CCR2 //0h
//#define mor_luaBanh_back          		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET) //phai -> trai
//#define mor_luaBanh_next 						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET)  //trai -> phai

#define mor_cuonGiua									TIM5->CCR2 //0h
#define mor_cuonGiua_back          		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET) //phai -> trai
#define mor_cuonGiua_next 						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET)  //trai -> phai

//---------------------------

#define mor_0hRo 								TIM5->CCR3 // 
#define mor_0hRo_next           HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET) 
#define mor_0hRo_back 					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET)  


#define mor_0h 								TIM5->CCR4 //
#define mor_0h_next        		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET) 
#define mor_0h_back 					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET)

//#define mor_thaCelo									TIM15->CCR1 // sau phai
//#define mor_thaCelo_back  						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET) 
//#define mor_thaCelo_next							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET)

#define mor_cuonTren							TIM15->CCR1 // sau phai
#define mor_cuonTren_back  				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET) 
#define mor_cuonTren_next					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET)  

//#define mor_4h									TIM15->CCR1 // sau phai
//#define mor_4h_back  						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET) 
//#define mor_4h_next							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET)  


//#define mor_luaBanh									TIM15->CCR2 // sau trai
//#define mor_luaBanh_next   					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET) 
//#define mor_luaBanh_back 						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET)

//#define mor_keo_phai									TIM15->CCR2 // sau trai
//#define mor_keo_phai_next   					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET) 
//#define mor_keo_phai_back 						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET)

#define mor_keo_trai									TIM12->CCR1 // sau trai
#define mor_keo_trai_next   					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET) 
#define mor_keo_trai_back 						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET)

#define mor_keo_phai									TIM12->CCR2 // sau trai
#define mor_keo_phai_next   					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET) 
#define mor_keo_phai_back 						HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET)

 //----------------------- TIM8-CH3 ------------------------
#define CB_Xilanh_ChamThanh_Phai HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) 
#define CB_Xilanh_ChamThanh_Trai HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) 

 //----------------------- I2C2-Input ------------------------
#define CB_CoBong_PhuGiua 	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) 
#define CBXXX 							HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) 

//----------------------- BUS 10 IO-1 INPUT------------------------
#define CB_4h HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) 
#define CB_0h HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) 
#define CB_8h HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) 
#define CB_NhanBanh3 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) 
//#define IO_D4 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) 
#define CB_NhanBanh2 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) 
//#define IO_D5 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) 
#define CB_NhanBanh1 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) 
#define CB_Bong_Tren HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) 
#define HanhTrinhSauTren HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) 

#define RED_BLUE	 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) // HIGH IS BLUE - LOW IS RED

//----------------------- BUS 10 IO2- INPUT--------------------------
// PIN 0V
// PIN 3,3V/5V
#define CB_Bong_Trai HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) 
#define CB_Bong_Duoi HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) 
#define CB_ThocPhai HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) 		// Cam bien mau
#define CB_GiuaCeilo HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) 
#define CB_Bong_Giua HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) 				
#define CB_Bong_Phai HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) 	// Cam bien giua nhan bong
#define CB_CoBong HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) 
#define CBMauTrai HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_14) 

int count_CB_Bong_Duoi;
char BongDuoi;
///////////////////////////////////////////////////////////////////////////
#define nhanMau HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)
#define CBbanhCheRobotTrai		(inputData & 0x0020)	
#define CBbanhCheRobotPhai		(inputData & 0x0040)	
#define vungVang1							(inputData & 0x0001) 	//Trai sang phai
#define vungVang2 						(inputData & 0x0002) 	
#define vungVang3 						(inputData & 0x0004) 	
#define vungVang4							(inputData & 0x0008)	
#define nutVang								(inputData & 0x0080)	//CB8
#define nutDo									HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)	//CB8
#define nutXanh									HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)	//CB8
//#define CB8 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)
//#define CB9 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6)
//#define CB10 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5)
//#define CB11 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)
//#define CB12 HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3)

//------------------------BUS 3 ------------------
//#define nutLayVongTrai  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) //day kep ra
//#define nutLayVongPhai 	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1)
//#define Nut_ResetAll 		HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)

//#define XL_1_ON  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET) //day kep ra
//#define XL_1_OFF HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET)

//#define XL_2_ON  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET) //kep
//#define XL_2_OFF HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET)

//#define XL_3_ON  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET) //Fire arrow
//#define XL_3_OFF HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET)

//#define XL_4_ON  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET) //chua sai
//#define XL_4_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET)

//#define XL_5_ON  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET) // chua sai
//#define XL_5_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET) // chua sai

//#define XL_6_ON  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_SET) // chua sai 
//#define XL_6_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET) // chua sai
/*
#define XL_1_ON  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET)
#define XL_1_OFF HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET)

#define XL_1_ON  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET)
#define XL_1_OFF HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET)

// IO pin bus 1: C0 -> C3,E1,E2 //IN
// IO pin bus 2: D3 -> D7,E0  // IN
// IO pin bus 3: D2,D10,D0,C12,C11,C10 // OUT
// 9 Input
// 

*/



//  ************ KHAI BAO NGO RA CHO 595  *****************
// Chu y khi sua ten ngo ra chi sua 0h chu "on/off" de dam bao dung trang thai on off
// Vi du sua OUT_1_on thanh xilanh_kep_on

//===================================================
//#define en_cam readEncoder_Tim4

//////////////////////////////////////////////////
//===========Khai bao cac bien PID cho reset tay danh===========
float P_rgocquay;
float Er_rgocquay;
//==============================================================
	int Uptest;
	int Downtest;
	int run, ban;
	int16_t pos;
uint8_t speed;
uint16_t gocquay, dem, j, i;

//--------------------------- Read Encoder ----------------------------//
//extern volatile int32_t num_over_t1;

//int32_t readEncoderTim_1() //TIMER 1
//{
//	return ((num_over_t1<<16)|TIM1->CNT);  // giam so xung encoder xuong 1 nua
//}

