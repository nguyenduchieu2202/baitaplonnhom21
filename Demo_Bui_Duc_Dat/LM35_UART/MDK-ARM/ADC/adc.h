/**
  ******************************************************************************
  * @file		adc.h
  * @author	www.hocdientu123.vn
  * @date		25/06/2019
  ******************************************************************************
  */
	
#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
 extern "C" {
#endif
//----------KHAI BAO CAC HAM CHO ADC.C
#include "main.h"	
void ADC_Configuration(void);
uint16_t ADC0_Read(void);
#ifdef __cplusplus
}
#endif

#endif
