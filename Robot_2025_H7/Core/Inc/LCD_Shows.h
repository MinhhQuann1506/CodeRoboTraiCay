#include <core_cm7.h>
#include "i2c_lcd.h"
#include "ala_Config.h"

extern UART_HandleTypeDef huart1;
//extern char kiemTraTrinhTuTimThoc;
//extern int dinhThoi;
extern int IMU;
extern uint16_t adc_data[11];
extern int16_t ball_x, ball_y, hoop_x, hoop_y;
extern char GP_BTN [10];
extern char MANG_GAME[8];
//extern unsigned char GP_BTN [10];
//extern uint32_t _ADC[8],_ADC_SUM[8];
//extern int16_t IMU;
//extern int16_t temp;
//extern TIM_HandleTypeDef htim3;
//extern int select_pole;
//extern int current_speed, current_angle;
//extern int32_t lazer_Truoc, lazer_Trai, lazer_Phai;
//extern int soChuyenTru;
//extern int demSoLanNhanO, demSoLanNhanSQ;
//extern uint32_t demNutNhanO;
//extern int sanxanh;
//extern char demCoCau;
////===================Khai bao cac bien cho bo banh xe======================
//extern uint8_t RX_UART1[2], RX_UART2[15], RX_UART3[1], RX_UART4[10], RX_UART5[1];
//extern uint8_t MANG_GAME[8];
//extern char bitResetBanhXe, bitTest;
//extern int gocQuayBanh, runLayLua;
//extern int speedGocQuay8h, erGocQuay8h, lastErGocQuay8h, KdGocQuay8h, KiGocQuay8h, KpGocQuay8h;
//extern int speedGocQuay4h, erGocQuay4h, lastErGocQuay4h, KdGocQuay4h, KiGocQuay4h, KpGocQuay4h;
//extern int speedGocQuay0hTrai, erGocQuay0hTrai, lastErGocQuay0hTrai, KdGocQuay0hTrai, KiGocQuay0hTrai, KpGocQuay0hTrai;
//extern int speedGocQuay0hPhai, erGocQuay0hPhai, lastErGocQuay0hPhai, KdGocQuay0hPhai, KiGocQuay0hPhai, KpGocQuay0hPhai;

//extern int AlphaC1, AlphaC2, AlphaC3;
//extern int Gamma1, Gamma2, Gamma3;
//extern float SinGamma1, SinGamma2, SinGamma3;
//extern float CosGamma1, CosGamma2, CosGamma3;
//extern float Rcp;
//extern float R1a, R2a, R3a, Rmax;
//extern float Delta1, Delta2, Delta3;
//extern float Alpha1, Alpha2, Alpha3;
//extern float V1, V2, V3;
//==================Khai bao trong 3SwerveWheel.h=======================
//extern int erRobotRun;
char buff[33];
//======================================================================
void lcdViewer( int mode)
{
		if(RED_BLUE)		lcd_printStr(18,0,"XA");
		else lcd_printStr(18,0,"DO");
	if (mode == 0) //Test tat ca cac thong so
	{
		lcd_printStr(0,0, "Data: ");
		lcd_printInt(5,0, RX_UART4[0]);
		lcd_printInt(10,0, RX_UART4[1]);
		lcd_printInt(15,0, RX_UART4[2]);
		lcd_printInt(20,0, RX_UART4[3]);
		lcd_printInt(5,1, RX_UART4[4]);
		lcd_printInt(10,1, RX_UART4[5]);
		lcd_printInt(15,1, RX_UART4[6]);
		lcd_printInt(20,1, RX_UART4[7]);
		lcd_printInt(5,2, RX_UART4[8]);
		lcd_printInt(10,2, RX_UART4[9]);
		lcd_printInt(0,3, ball_x);
		lcd_printInt(5,3, ball_y);
		lcd_printInt(10,3, hoop_x);
		lcd_printInt(15,3, hoop_y);
	}
	
	else if (mode == 1) //Test encoder
	{
		lcd_printStr(0,0,"M1");
	}
	else if(mode == 2){
		lcd_printStr(0,0,"M2");
	}
	else if(mode == 3){ // Test doc mau celo
		lcd_printStr(0,0,"M3");
	}
	else if(mode == 4){ //Test nut tay game
		lcd_printStr(0,0,"M4");
	}
	
	else if(mode == 5){ //Test encoder
		lcd_printStr(0,0,"M5 Thong So Xoay");
		
//		lcd_printStr(0,1,"SS: ");
//		lcd_printBit(CB_2h);
//		lcd_printBit(CB_4h);
//		lcd_printBit(CB_8h);
//		lcd_printBit(CB_10h);
		lcd_printStr(0,1,"mhX:");
		lcd_printInt(4,1,en_mamXoay());
		lcd_printStr(8,1,"2XC:");
		lcd_printInt(12,1,en_Chay2h());
		lcd_printStr(0,2,"8hX:");
		lcd_printInt(4,2,en_8hRo());
		lcd_printStr(0,3,"4hX:");
		lcd_printInt(4,3,en_4hRo());
		lcd_printStr(8,2,"10X:");
		lcd_printInt(12,2,en_10hRo());
		lcd_printStr(8,3,"2hX:");
		lcd_printInt(12,3,en_2hRo());
		
	}
	
	else if(mode == 6) { // test cam bien Input
		lcd_printStr(0,0,"M6 TEST GAMEPAD DATA");
		
		lcd_printInt(0,1,GP_BTN[0]);
		lcd_printInt(5,1,GP_BTN[1]);
		lcd_printInt(10,1,(int8_t)GP_BTN[2]);
		lcd_printInt(15,1,(int8_t)GP_BTN[3]);
		lcd_printInt(0,2,(int8_t)GP_BTN[4]);
		lcd_printInt(5,2,(int8_t)GP_BTN[5]);
		lcd_printInt(10,2,GP_BTN[6]);
		lcd_printInt(15,2,GP_BTN[7]);
		
		lcd_printInt(0,3,en_mamXoay());
	}
	else if(mode == 7) { // test adc
		lcd_printStr(0,0,"M7 TEST ADC");
		
		lcd_printInt(0,1,adc_data[0]);	
		lcd_printInt(5,1,adc_data[1]);
		lcd_printInt(10,1,adc_data[2]);
		
		lcd_printInt(0,2,adc_data[3]);	
		lcd_printInt(5,2,adc_data[4]);
		lcd_printInt(10,2,adc_data[5]);
		lcd_printInt(15,2,adc_data[6]);
		
		lcd_printInt(0,3,adc_data[7]);	
		lcd_printInt(5,3,adc_data[8]);
		lcd_printInt(10,3,adc_data[9]);
		lcd_printInt(15,3,adc_data[10]);
	}
	else {
		lcd_printStr(0,0,"M8");
		
	}
}


