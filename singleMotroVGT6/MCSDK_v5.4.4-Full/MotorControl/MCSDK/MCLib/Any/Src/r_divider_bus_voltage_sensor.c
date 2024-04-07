/**
  ******************************************************************************
  * @file    r_divider_bus_voltage_sensor.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the  features
  *          of the Resistor Divider Bus Voltage Sensor component of the Motor
  *          Control SDK:
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "r_divider_bus_voltage_sensor.h"
#include "regular_conversion_manager.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup BusVoltageSensor
  * @{
  */

/** @defgroup RDividerBusVoltageSensor Resistor Divider Bus Voltage Sensor
  * @brief Resistor Divider Bus Voltage Sensor implementation
  *
  * @todo Document the Resistor Divider Bus Voltage Sensor "module".
  *
  * @{
  */

/**
  * @brief  It initializes bus voltage conversion (ADC, ADC channel, conversion time. 
    It must be called only after PWMC_Init.
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
__weak void RVBS_Init( RDivider_Handle_t * pHandle )
{
  /* Need to be register with RegularConvManager */
  pHandle->convHandle = RCM_RegisterRegConv(&pHandle->VbusRegConv);
  /* Check */
  RVBS_Clear( pHandle );
}


/**
  * @brief  It clears bus voltage FW variable containing average bus voltage
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval none
  */
__weak void RVBS_Clear( RDivider_Handle_t * pHandle )
{
  uint16_t aux;
  uint16_t index;
//把所有的电压滤波变量清除为过压值和欠压值的平均值
  aux = ( pHandle->OverVoltageThreshold + pHandle->UnderVoltageThreshold ) / 2u;
  for ( index = 0u; index < pHandle->LowPassFilterBW; index++ )
  {
    pHandle->aBuffer[index] = aux;
  }
  pHandle->_Super.LatestConv = aux;
  pHandle->_Super.AvBusVoltage_d = aux;
  pHandle->index = 0;
}

static uint16_t RVBS_ConvertVbusFiltrered( RDivider_Handle_t * pHandle )
{
  uint16_t hAux;
  uint8_t vindex;
  uint16_t max = 0, min = 0;
  uint32_t tot = 0u;

  for ( vindex = 0; vindex < pHandle->LowPassFilterBW; )
  {
    hAux = RCM_ExecRegularConv(pHandle->convHandle);//总线电压是规则转换 这里是读取总线电压值
  //找出LowPassFilterBW次滤波的最大值和最小值，并将所有的读取的hAux值相加 去掉最大值和最小值 在平局
    if ( hAux != 0xFFFFu )
    {
      if ( vindex == 0 )
      {
        min = hAux;
        max = hAux;
      }
      else
      {
        if ( hAux < min )
        {
          min = hAux;
        }
        if ( hAux > max )
        {
          max = hAux;
        }
      }
      vindex++;

      tot += hAux;
    }
  }

  tot -= max;//去掉最大值和最小值
  tot -= min;
  return ( uint16_t )( tot / ( pHandle->LowPassFilterBW - 2u ) );//减去最大值和最小值2次滤波计数
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
//参照这个函数RVBS_CalcAvVbus( RDivider_Handle_t * pHandle ) 注释
__weak uint16_t RVBS_CalcAvVbusFilt( RDivider_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = RVBS_ConvertVbusFiltrered( pHandle );//读取总线电压返回滤波值

  if ( hAux != 0xFFFF )
  {
    pHandle->aBuffer[pHandle->index] = hAux;
    wtemp = 0;
    for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
    {
      wtemp += pHandle->aBuffer[i];
    }
    wtemp /= pHandle->LowPassFilterBW;
    pHandle->_Super.AvBusVoltage_d = ( uint16_t )wtemp;
    pHandle->_Super.LatestConv = hAux;

    if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
    {
      pHandle->index++;
    }
    else
    {
      pHandle->index = 0;
    }
  }

  pHandle->_Super.FaultState = RVBS_CheckFaultState( pHandle );

  return ( pHandle->_Super.FaultState );
}

/**
  * @brief  It actually performes the Vbus ADC conversion and updates average
  *         value
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
__weak uint16_t RVBS_CalcAvVbus( RDivider_Handle_t * pHandle )
{
  uint32_t wtemp;
  uint16_t hAux;
  uint8_t i;

  hAux = RCM_ExecRegularConv(pHandle->convHandle);

  if ( hAux != 0xFFFF )
  {
		//函数每执行一次更新aBuffer[pHandle->index]中的一位（将总线电压hAux赋值给aBuffer[pHandle->index）（函数每执行一次pHandle->index累加1）
    //程序刚开始执行的时候abuffer中都为0 随着执行次数的增多 滤波值wtemp逐渐等于实际值
		pHandle->aBuffer[pHandle->index] = hAux;//更新数字中的某一个
    wtemp = 0;
    for ( i = 0; i < pHandle->LowPassFilterBW; i++ )
    {
      wtemp += pHandle->aBuffer[i];//将所有记录下的电压总线值累加
    }
    wtemp /= pHandle->LowPassFilterBW;//求平均值
    pHandle->_Super.AvBusVoltage_d = ( uint16_t )wtemp;//滤波后赋值给结构体总线电压变量
    pHandle->_Super.LatestConv = hAux;//记录上次一读取总线的电压值

    if ( pHandle->index < pHandle->LowPassFilterBW - 1 )
    {
      pHandle->index++;//滤波次数没到 滤波次数加1  为了赋值下一个数组值
    }
    else
    {
      pHandle->index = 0;//滤波次数到了，清0 
    }
  }

  pHandle->_Super.FaultState = RVBS_CheckFaultState( pHandle );//检测是否过压或欠压

  return ( pHandle->_Super.FaultState );
}

/**
  * @brief  It returns MC_OVER_VOLT, MC_UNDER_VOLT or MC_NO_ERROR depending on
  *         bus voltage and protection threshold values
  * @param  pHandle related RDivider_Handle_t
  * @retval uint16_t Fault code error
  */
__weak uint16_t RVBS_CheckFaultState( RDivider_Handle_t * pHandle )
{
  uint16_t fault;

  if ( pHandle->_Super.AvBusVoltage_d > pHandle->OverVoltageThreshold ) //检测是否过压
  {
    fault = MC_OVER_VOLT;
  }
  else if ( pHandle->_Super.AvBusVoltage_d < pHandle->UnderVoltageThreshold )//检测是否欠压
  {
    fault = MC_UNDER_VOLT;
  }
  else
  {
    fault = MC_NO_ERROR;
  }
  return fault;
}


/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

