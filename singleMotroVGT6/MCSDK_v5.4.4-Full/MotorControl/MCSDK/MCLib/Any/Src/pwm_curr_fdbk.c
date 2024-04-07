/**
  ******************************************************************************
  * @file    pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * current sensing
  *           * regular ADC conversion execution
  *           * space vector modulation
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
#include "pwm_curr_fdbk.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup pwm_curr_fdbk PWM & Current Feedback
  *
  * @brief PWM & Current Feedback components of the Motor Control SDK
  *
  * These components fulfill two functions in a Motor Control subsystem:
  *
  * - The generation of the Space Vector Pulse Width Modulation on the motor's phases
  * - The sampling of the actual motor's phases current
  *
  * Both these features are closely related as the instants when the values of the phase currents
  * should be sampled by the ADC channels are basically triggered by the timers used to generate
  * the duty cycles for the PWM.
  *
  * Several implementation of PWM and Current Feedback components are provided by the Motor Control
  * SDK to account for the specificities of the application:
  *
  * - The selected MCU: the number of ADCs available on a given MCU, the presence of internal
  * comparators or OpAmps, for instance, lead to different implementation of this feature
  * - The Current sensing topology also has an impact on the firmware: implementations are provided
  * for Insulated Current Sensors, Single Shunt and Three Shunt resistors current sensing topologies
  *
  * The choice of the implementation mostly depend on these two factors and is performed by the
  * Motor Control Workbench tool.
  *
  * All these implementations are built on a base PWM & Current Feedback component that they extend
  * and that provides the functions and data that are common to all of them. This base component is
  * never used directly as it does not provide a complete implementation of the features. Rather,
  * its handle structure (PWMC_Handle) is reused by all the PWM & Current Feedback specific
  * implementations and the functions it provides form the API of the PWM and Current feedback feature.
  * Calling them results in calling functions of the component that actually implement the feature.
  * See PWMC_Handle for more details on this mechanism.
  * @{
  */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief Returns the phase current of the motor as read by the ADC (in s16A unit)
  *
  * The function actually returns the current values of phase A & B. Phase C current
  * can be deduced thanks to the formula:
  *
  * @f[
  * I_{C} = -I_{A} - I_{C}
  * @f]
  *
  * @param  pHandle handle on the target PWMC component
  * @param  pStator_Currents Pointer to the structure that will receive motor current
  *         of phase A and B in ElectricalValue format.
*/
__weak void PWMC_GetPhaseCurrents( PWMC_Handle_t * pHandle, ab_t * Iab )
{
  pHandle->pFctGetPhaseCurrents( pHandle, Iab );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
 * @brief Sets the PWM duty cycles
 *
 *
 */

/**
  * @brief  Converts input voltages @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into PWM duty cycles
  *         and feed them to the inverter.
  * @param  pHandle handler on the target PWMC component.
  * @param  Valfa_beta Voltage Components expressed in the @f$(\alpha, \beta)@f$ reference frame
  *
  * This function computes the the time during wIch the transistors of each phase are to be switched on in
  * a PWM cycle in order to achieve the reference phase voltage set by @p Valfa_beta. Then, the function
  * programs the resulting duty cycles in the related timer channels. It also sets the phase current
  * sampling point for the next PWM cycle accordingly.
  *
  * This function is used in the FOC frequency loop and needs to complete before the next PWM cycle starts
  * so that the duty cycles it computes can be taken into account. Failing to do so (for instance because
  * the PWM Frequency is too high) results in the functions returning #MC_FOC_DURATION wIch entails a
  * Motor Control Fault that stops the motor.
  *
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_FOC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__weak uint16_t PWMC_SetPhaseVoltage( PWMC_Handle_t * pHandle, alphabeta_t Valfa_beta )
{
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
  //.hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,//?????
	//#define SQRT3FACTOR (uint16_t) 0xDDB4 /* = (16384 * 1.732051 * 2)*/
	//�����һ���� 0<arctan(U��/U��)<60  U�� > 0,U�� > 0
	//ֻҪ��U�� U�� �ϳɵ�ʸ�������Ƶ�ÿ��60�������ڣ��γɵ��������� �����ж���������������  
  wUAlpha = Valfa_beta.alpha * ( int32_t )pHandle->hT_Sqrt3;//U��*T*2*1.732
  wUBeta = -( Valfa_beta.beta * ( int32_t )( pHandle->PWMperiod ) ) * 2;// -2*U��*T
  //�ͳ��ж��������������� ��׼��ʽ
	//U1 = U��
	//U2 = (��3/2)*U�� - U��/2
	//U3 = -(��3/2)*U�� - U��/2
	// ����������2��
	//ע�⣺�����������������������������������и��ĵ�дU�� �е��� -U��
	//����ΪST���������̵ı�̬����Y�Ḻ����Ϊb�������� ����û��Ӱ��
  wX = wUBeta;//-2*U��*T 
  wY = ( wUBeta + wUAlpha ) / 2;//(U��*T*2*1.732 - 2*T*U��)/2
  wZ = ( wUBeta - wUAlpha ) / 2;//(-U��*T*2*1.732 - 2*T*U��)/2

  /* Sector calculation from wX, wY, wZ */
  if ( wY < 0 )
  {
    if ( wZ < 0 )
    {
      pHandle->Sector = SECTOR_5;
			//����������ʱ��X Y Z ������ T1 T2ʱ�� 
			//����			T1		T2	
			//I	    	  Y  	 -X		
			//II	  	 -Y		 -Z		
			//III			 -X			Z	
			//IV			  X		 -Y		
			//V				  Z		  Y	
			//VI			 -Z		 -Y	
 /*******************����ÿ������A��ռ�ձ�ʱ��********************
			����1��6��A��ʱ�� TA=(T+T1+T2)/4 
			����2��5��A��ʱ�� TA=(T+T2-T1)/4
			����3��4��A��ʱ�� TA=(T-T1-T2)/4
			B���C�� ͨ��A���T1/2����T2/2 ��ӻ�� 
			�������ο�ÿһ��������PWM����ͼ
 ****************************************************************/
			//�����Ǽ�����ʽ
			//Ta = T/2 - (T - Tx - Ty)/4 -----��1��
			//T :PWM����
			//T/2:�������Ķ����CNT (0-CNT-0)
			//(T - Tx - Ty)/4 �������ڼ�ȥ����ʸ��ʣ�µľ���0ʸ��������Ҫ������0ʸ������ʱ����ͬ��
			//��Ϊÿ�������ĵ��Ʋ������ĶԳƵ� ������0ʸ���ֳ�4���� ���Գ���4
			//����1���� Ta = ��T + Tx + Ty��/4
			//����( int32_t )( pHandle->PWMperiod ) / 4 ����4������
			//( ( wY - wZ ) / ( int32_t )262144 ) = ( ( wY - wZ ) / 	q15 * 2 * 4 )  
			//q15 Ϊ���õĸ�ʽ 2������wx wy wz ��ʱ��Ŵ��2�� ���Գ�2
      wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
			//wZ / 131072 = wZ / q15 * 2 *2 
			// ��һ��2 �ǷŴ���2���ڼ���WZ��ʱ�����Գ�2 
			//�ڶ���2�������Ǽ���ʱ��ಿ�֣�ÿһ��ʸ������ʱ������ʱ���һ�� �������WX WY WZ ����һ��pwm������ÿһ�ַ���ʸ�����õ���Чʱ�䣩
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
			//�����¹ܿ���ʱ��ĵĴ�С��ע�����¹ܿ���ʱ�䣩
      pHandle->lowDuty = wTimePhC;//��¼��һ��ʱ����̣�������������У��������¹ܿ���ʱ����̵�һ�࣬�û�������ϳɣ�Ϊ�˲�����׼ȷ
      pHandle->midDuty = wTimePhA;
      pHandle->highDuty = wTimePhB;
    }
    else /* wZ >= 0 */
      if ( wX <= 0 )
			{ 
        pHandle->Sector = SECTOR_4;
				//����������A��ռ�ձ�
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
				//����������B��ռ�ձ�
        wTimePhB = wTimePhA + wZ / 131072;
				//����������C��ռ�ձ�
        wTimePhC = wTimePhB - wX / 131072;
        pHandle->lowDuty = wTimePhC;
        pHandle->midDuty = wTimePhB;
        pHandle->highDuty = wTimePhA;
      }
      else /* wX > 0 */
      {
        pHandle->Sector = SECTOR_3;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pHandle->lowDuty = wTimePhB;
        pHandle->midDuty = wTimePhC;
        pHandle->highDuty = wTimePhA;
      }
  }
  else /* wY > 0 */
  {
    if ( wZ >= 0 )
    {
      pHandle->Sector = SECTOR_2;
      wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      pHandle->lowDuty = wTimePhB;
      pHandle->midDuty = wTimePhA;
      pHandle->highDuty = wTimePhC;
    }
    else /* wZ < 0 */
      if ( wX <= 0 )
      {
        pHandle->Sector = SECTOR_6;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        pHandle->lowDuty = wTimePhA;
        pHandle->midDuty = wTimePhC;
        pHandle->highDuty = wTimePhB;
      }
      else /* wX > 0 */
      {
        pHandle->Sector = SECTOR_1;
        wTimePhA = ( int32_t )( pHandle->PWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        pHandle->lowDuty = wTimePhA;
        pHandle->midDuty = wTimePhB;
        pHandle->highDuty = wTimePhC;
      }
  }

  pHandle->CntPhA = ( uint16_t )wTimePhA;
  pHandle->CntPhB = ( uint16_t )wTimePhB;
  pHandle->CntPhC = ( uint16_t )wTimePhC;

	//��������ԭ���ȿ�����ĳһ������ű� ����������ʱ�¹ܵ�ͨ����ǰ��ѹ��ͬһ�ű�����mos��֮���ѹ��
	//Ϊ�͵�ƽ��ĳһʱ���¹ܹضϣ����û������������£���������¹ܹ��Ϲ����̿�ͨ����ǰ��ѹ���̱��VD(�����ѹ)
	//�ڴ������������������������ܵ����������ã���ǰ��ѹ��ȻΪGND ,�������������Ƚϱ�����������������������ѹ��С
	//����Ӧ�ò�����Ӧ�ĵ�ѹֵ�����ӵĿ�ͨʱ��䳤�ı� pHandle->CntPhA��ֵ  ��������ֶ�����һ���̶�����ֵ����
	// �Ϲ�ԭ����ͬ �ȿ���ͨ״̬����������ڹر�ʱ ͬһ���ű���mos�м���ѹGND ������������ ��ȻΪVD ������Ҫ���ٹ��ӵ�ͨʱ��
  if ( pHandle->DTTest == 1u )//ʹ����������
  {
    /* Dead time compensation */
    if ( pHandle->Ia > 0 )
    {
      pHandle->CntPhA += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhA -= pHandle->DTCompCnt;
    }

    if ( pHandle->Ib > 0 )
    {
      pHandle->CntPhB += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhB -= pHandle->DTCompCnt;
    }

    if ( pHandle->Ic > 0 )
    {
      pHandle->CntPhC += pHandle->DTCompCnt;
    }
    else
    {
      pHandle->CntPhC -= pHandle->DTCompCnt;
    }
  }

  return ( pHandle->pFctSetADCSampPointSectX( pHandle ) );//���õ���������
}

/**
  * @brief  Switches PWM generation off, inactivating the outputs.
  * @param  pHandle Handle on the target instance of the PWMC component
  */
__weak void PWMC_SwitchOffPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm( pHandle );
}

/**
  * @brief  Switches PWM generation on
  * @param  pHandle Handle on the target instance of the PWMC component
  */
__weak void PWMC_SwitchOnPWM( PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm( pHandle );
}

/**
  * @brief  Calibrates ADC current conversions by reading the offset voltage
  *         present on ADC pins when no motor current is flowing in.
  *
  * This function should be called before each motor start-up.
  *
  * @param  pHandle Handle on the target instance of the PWMC component
  * @param  action Can be #CRC_START to initialize the offset calibration or
  *         #CRC_EXEC to execute the offset calibration.
  * @retval true if the current calibration has been completed, false if it is
  *         still ongoing.
  */
__weak bool PWMC_CurrentReadingCalibr( PWMC_Handle_t * pHandle, CRCAction_t action )
{
  bool retVal = false;
  if ( action == CRC_START )
  {
    PWMC_SwitchOffPWM( pHandle );
    pHandle->OffCalibrWaitTimeCounter = pHandle->OffCalibrWaitTicks;
    if ( pHandle->OffCalibrWaitTicks == 0u )
    {
      pHandle->pFctCurrReadingCalib( pHandle );
      retVal = true;
    }
  }
  else if ( action == CRC_EXEC )
  {
    if ( pHandle->OffCalibrWaitTimeCounter > 0u )
    {
      pHandle->OffCalibrWaitTimeCounter--;
      if ( pHandle->OffCalibrWaitTimeCounter == 0u )
      {
        pHandle->pFctCurrReadingCalib( pHandle );
        retVal = true;
      }
    }
    else
    {
      retVal = true;
    }
  }
  else
  {
  }
  return retVal;
}

/**
  * @brief  Switches power stage Low Sides transistors on.
  *
  * This function is meant for charging boot capacitors of the driving
  * section. It has to be called on each motor start-up when using high
  * voltage drivers.
  *
  * @param  pHandle: handle on the target instance of the PWMC component
  */
__weak void PWMC_TurnOnLowSides( PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides( pHandle );
}


/** @brief Returns #MC_BREAK_IN if an over current condition was detected on the power stage
 *         controlled by the PWMC component pointed by  @p pHandle, since the last call to this function;
 *         returns #MC_NO_FAULTS otherwise. */
__weak uint16_t PWMC_CheckOverCurrent( PWMC_Handle_t * pHandle )
{
  return pHandle->pFctIsOverCurrentOccurred( pHandle );
}

/**
  * @brief  Sets the over current threshold to be used
  *
  * The value to be set is relative to the VDD_DAC DAC reference voltage with
  * 0 standing for 0 V and 65536 standing for VDD_DAC.
  *
  * @param  pHandle handle on the target instance of the PWMC component
  * @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer
  */
__weak void PWMC_OCPSetReferenceVoltage( PWMC_Handle_t * pHandle, uint16_t hDACVref )
{
  if ( pHandle->pFctOCPSetReferenceVoltage )
  {
    pHandle->pFctOCPSetReferenceVoltage( pHandle, hDACVref );
  }
}

/**
  * @brief  It is used to retrieve the satus of TurnOnLowSides action.
  * @param  pHandle: handler of the current instance of the PWMC component
  * @retval bool It returns the state of TurnOnLowSides action:
  *         true if TurnOnLowSides action is active, false otherwise.
  */
/** @brief Returns the status of the "TurnOnLowSide" action on the power stage
 *         controlled by the @p pHandle PWMC component: true if it
 *         is active, false otherwise*/
__weak bool PWMC_GetTurnOnLowSidesAction( PWMC_Handle_t * pHandle )
{
  return pHandle->TurnOnLowSidesAction;
}

/** @brief Enables the RL detection mode on the power stage controlled by the @p pHandle PWMC component. */
__weak void PWMC_RLDetectionModeEnable( PWMC_Handle_t * pHandle )
{
  if ( pHandle->pFctRLDetectionModeEnable )
  {
    pHandle->pFctRLDetectionModeEnable( pHandle );
  }
}

/** @brief Disables the RL detection mode on the power stage controlled by the @p pHandle PWMC component. */
__weak void PWMC_RLDetectionModeDisable( PWMC_Handle_t * pHandle )
{
  if ( pHandle->pFctRLDetectionModeDisable )
  {
    pHandle->pFctRLDetectionModeDisable( pHandle );
  }
}

/**
  * @brief  Sets the PWM duty cycle to apply in the RL Detection mode.
  * @param  pHandle: handle on the target instance of the PWMC component
  * @param  hDuty Duty cycle to apply
  *
  * @todo TODO: Describe the unit of the hDuty variable.
  *
  * @retval If the Duty Cycle could be applied on time for the next PWM period,
  *         #MC_NO_ERROR is returned. Otherwise, #MC_FOC_DURATION is returned.
  */
__weak uint16_t PWMC_RLDetectionModeSetDuty( PWMC_Handle_t * pHandle, uint16_t hDuty )
{
  uint16_t hRetVal = MC_FOC_DURATION;
  if ( pHandle->pFctRLDetectionModeSetDuty )
  {
    hRetVal = pHandle->pFctRLDetectionModeSetDuty( pHandle, hDuty );
  }
  return hRetVal;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to get phases current.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterGetPhaseCurrentsCallBack( PWMC_GetPhaseCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctGetPhaseCurrents = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation off.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterSwitchOffPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                        PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOffPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to switch PWM
 *        generation on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterSwitchonPwmCallBack( PWMC_Generic_Cb_t pCallBack,
                                       PWMC_Handle_t * pHandle )
{
  pHandle->pFctSwitchOnPwm = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to execute a calibration
 *        of the current sensing system.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterReadingCalibrationCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctCurrReadingCalib = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to turn low sides on.
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterTurnOnLowSidesCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctTurnOnLowSides = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to compute ADC sampling point
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterSampPointSectXCallBack( PWMC_SetSampPointSectX_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctSetADCSampPointSectX = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to the overcurrent status
 * @param pCallBack pointer on the callback
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterIsOverCurrentOccurredCallBack( PWMC_OverCurr_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctIsOverCurrentOccurred = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the reference
 *        voltage for the over current protection
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterOCPSetRefVoltageCallBack( PWMC_SetOcpRefVolt_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctOCPSetReferenceVoltage = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to enable the R/L detection mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterRLDetectionModeEnableCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeEnable = pCallBack;
}

/**
 * @brief Sets the Callback wIch PWMC shall invoke to disable the R/L detection mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterRLDetectionModeDisableCallBack( PWMC_Generic_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeDisable = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to set the duty cycle
 *        for the R/L detection mode
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 */
__weak void PWMC_RegisterRLDetectionModeSetDutyCallBack( PWMC_RLDetectSetDuty_Cb_t pCallBack,
    PWMC_Handle_t * pHandle )
{
  pHandle->pFctRLDetectionModeSetDuty = pCallBack;
}

/**
 * @brief Sets the Callback that the PWMC component shall invoke to call PWMC instance IRQ handler
 * @param pHandle pointer on the handle structure of the PWMC instance
 *
 * @note this function is deprecated.
 */
__weak void PWMC_RegisterIrqHandlerCallBack( PWMC_IrqHandler_Cb_t pCallBack,
                                      PWMC_Handle_t * pHandle )
{
  pHandle->pFctIrqHandler = pCallBack;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
