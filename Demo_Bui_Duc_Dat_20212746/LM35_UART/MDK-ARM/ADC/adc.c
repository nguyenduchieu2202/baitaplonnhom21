#include "main.h"	
#include "adc.h"
/*******************************************************************************
Noi Dung      :   Khoi tao ADC
Tham Bien     :   Khong.
Tra Ve        :   Khong.
*******************************************************************************/
void ADC_Configuration()
{
		// Initialization struct
	ADC_InitTypeDef ADC_InitStruct;	
//cap xung clk cho bo ADC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
// Chon Che Do Cho Bo ADC
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
//Enable - Disable che do Scan
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
//Enable - Disable che do lien tuc
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
//Dinh nghia  Trigger ben ngoai de bat dau 1 chuyen doi ADC kenh Regular
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
// Enable kich thich chuyen doi tu ben ngoai
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
//Chon Kieu Luu Tru Du Lieu 
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
//Chon so luong kenh regular ADC su dung; cho phep 1 den 16
	ADC_InitStruct.ADC_NbrOfChannel = 1;
// Cau hinh ADC
	ADC_Init(ADC1, &ADC_InitStruct);
//Kich Hoat ADC1
	ADC_Cmd(ADC1, ENABLE);
//Bat Dau hieu chuan ADC1
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
 }
/*******************************************************************************
Noi Dung      :   Chon và doc gia tri ADC
Tham Bien     :   Khong.
Tra Ve        :   Khong.
*******************************************************************************/
uint16_t ADC0_Read()
{
//chon kenh chuyen doi ADC và so luong kenh chuyen doi, chu ky chuyen doi
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
// Start ADC conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
// Wait until ADC conversion finished
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
//tra ve gia tri ADC
	return (ADC_GetConversionValue(ADC1));
}
/********************************* END OF FILE ********************************/
