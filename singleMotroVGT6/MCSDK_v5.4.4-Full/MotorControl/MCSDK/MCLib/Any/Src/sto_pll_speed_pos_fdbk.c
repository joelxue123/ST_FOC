/**
  ******************************************************************************
  * @file    sto_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the State Observer + PLL Speed & Position Feedback component of the
  *          Motor Control SDK.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sto_pll_speed_pos_fdbk.h"
#include "mc_math.h"



/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @defgroup SpeednPosFdbk_STO State Observer Speed & Position Feedback
  * @brief State Observer with PLL Speed & Position Feedback implementation
  *
  * This component uses a State Observer coupled with a software PLL to provide an estimation of
  * the speed and the position of the rotor of the motor.
  *
  * @todo Document the State Observer + PLL Speed & Position Feedback "module".
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define C6_COMP_CONST1  (int32_t) 1043038
#define C6_COMP_CONST2  (int32_t) 10430

/* Private function prototypes -----------------------------------------------*/
static void STO_Store_Rotor_Speed( STO_PLL_Handle_t * pHandle, int16_t hRotor_Speed );
static int16_t STO_ExecutePLL( STO_PLL_Handle_t * pHandle, int16_t hBemf_alfa_est,
                               int16_t hBemf_beta_est );
static void STO_InitSpeedBuffer( STO_PLL_Handle_t * pHandle );


/**
  * @brief  It initializes the state observer component
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
__weak void STO_PLL_Init( STO_PLL_Handle_t * pHandle )
{
  int16_t htempk;
  int32_t wAux;


  pHandle->ConsistencyCounter = pHandle->StartUpConsistThreshold;
  pHandle->EnableDualCheck = true;

  wAux = ( int32_t )1;
  pHandle->F3POW2 = 0u;
//#define C6_COMP_CONST1  (int32_t) 1043038
//#define C6_COMP_CONST2  (int32_t) 10430
//#define F1                               16384
//#define F2                               4096
  htempk = ( int16_t )( C6_COMP_CONST1 / ( pHandle->hF2 ) );
//��htempk �����Լ����2��n�η� ����n��¼��������
  while ( htempk != 0 )
  {
    htempk /= ( int16_t )2;
    wAux *= ( int32_t )2;//htempk ���Ƴ�2��ָ����ʽ
    pHandle->F3POW2++;// htempk Ϊ���ٸ�2���
  }

  pHandle->hF3 = ( int16_t )wAux;
  wAux = ( int32_t )( pHandle->hF2 ) * pHandle->hF3;//��C6_COMP_CONST1��ԭ��ָ����ʽ 2��n�η�
  pHandle->hC6 = ( int16_t )( wAux / C6_COMP_CONST2 );

  STO_PLL_Clear( pHandle );

  PID_HandleInit( & pHandle->PIRegulator );

  /* Acceleration measurement set to zero */
  pHandle->_Super.hMecAccelUnitP = 0;

  return;
}

/**
  * @brief  It only returns, necessary to implement fictitious IRQ_Handler
  * @param  pHandle: handler of the current instance of the STO component
  * @param  uint8_t Fictitious interrupt flag
  * @retval none
  */

__weak void STO_PLL_Return( STO_PLL_Handle_t * pHandle, uint8_t flag )
{
  return;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This method executes Luenberger state observer equations and calls
  *         PLL with the purpose of computing a new speed estimation and
  *         updating the estimated electrical angle.
  * @param  pHandle: handler of the current instance of the STO component
  * @param  pInputVars_str pointer to the observer inputs structure
  * @retval int16_t rotor electrical angle (s16Degrees)
  */
/**
  * *****************************����������۲��������***********************************
  *���ݿ������ۣ����һ��ϵͳ�ܹ���ȫͨ�����⵽�����ֵ���ع���ϵͳ״̬������Ϊ��ϵͳ�ǿɹ۲�ģ�
  *����1 ʲô��������۲���
   ��ϵͳ���޷�ֱ�ӻ��ӻ�ȡ��Ҫ�Ĳ���������Ҫͨ������һ��ģ�ͣ��ѵ�ǰϵͳ������ͺ�����ʸ����Ϊ�½���ģ�͵�����
	 ������ģ�����������ֵΪ0��Ϊ��׼����Ϊ0ʱ֤������ϵͳ������ȣ��ѽ���ģ�͵Ĺ���ֵ��Ϊ��ϵͳ��ʵ��ֵ��ͬ
	*����2 ������ͻ�Ĥ������
	 ����ϵͳģ�ͻ�����ͬ����������������ԣ�������ֵ-ʵ��ֵ�� = 0����Ϊ��׼������ֵ��������ʵ��ֵ����ΪĿ�������Ϊ0��
   ��Ĥ���԰�������Ϊ��׼(������Ŀ��ֵ��ϵͳһ���̶���ֵ) ���磺������ʵ��ֵ��ϵͳһ���̶���ֵK,��С��ʵ��ֵ��-K
	 ��������ֵ����Ŀ��ֵ�������ذڶ��������ΪʲôҪ���е�ͨ�˲���ԭ����Ϊ��һ����ݲ����˲�����ƽ���Ĳ��Σ�
	 ����������һ�����������Ĳ��Σ���Ϊ������ֵ-ʵ��ֵ�� = 0��
	*����3 ����������۲��� 
	 Ϊʲô���ֹ۲���Ҫ���½�ģ���м���һ���仯������ʵ���Ͼ���������仯�����˶��켣ȥ����δ֪�����˶��켣��
  */
__weak int16_t STO_PLL_CalcElAngle( STO_PLL_Handle_t * pHandle, Observer_Inputs_t * pInputs )
{
  int32_t wAux, wDirection;
  int32_t wIalfa_est_Next, wIbeta_est_Next;
  int32_t wBemf_alfa_est_Next, wBemf_beta_est_Next;
  int16_t hAux, hAux_Alfa, hAux_Beta, hIalfa_err, hIbeta_err, hRotor_Speed,
          hValfa, hVbeta;

/**************************���綯��Ua����****************************************/
//#define F1                               16384
//#define F2                               4096
//#define F1_LOG                           LOG2(16384) = 15
//#define F2_LOG                           LOG2(4096) = 12
  if ( pHandle->wBemf_alfa_est > ( int32_t )( pHandle->hF2 )*INT16_MAX )//
  {
    pHandle->wBemf_alfa_est = INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else if ( pHandle->wBemf_alfa_est <= -INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_alfa_est = -INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else
  {
  }
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux_Alfa = ( int16_t )( pHandle->wBemf_alfa_est / pHandle->hF2 );
#else
  hAux_Alfa = ( int16_t )( pHandle->wBemf_alfa_est >> pHandle->F2LOG );//Ea�Ҳ��ƶ�12λ
#endif
/**************************���綯��Ub����****************************************/
  if ( pHandle->wBemf_beta_est > INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_beta_est = INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else if ( pHandle->wBemf_beta_est <= -INT16_MAX * ( int32_t )( pHandle->hF2 ) )
  {
    pHandle->wBemf_beta_est = -INT16_MAX * ( int32_t )( pHandle->hF2 );
  }
  else
  {
  }
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux_Beta = ( int16_t )( pHandle->wBemf_beta_est / pHandle->hF2 );
#else
  hAux_Beta = ( int16_t )( pHandle->wBemf_beta_est >> pHandle->F2LOG );//Ub�Ҳ��ƶ�12λ
#endif
//#define F1                               16384
//#define F2                               4096
//#define F1_LOG                           LOG2(16384) = 15
//#define F2_LOG                           LOG2(4096) = 12
	
/**************************����Ia����****************************************/
  if ( pHandle->Ialfa_est > INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ialfa_est = INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else if ( pHandle->Ialfa_est <= -INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ialfa_est = -INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else
  {
  }
/**************************����Ib����****************************************/
  if ( pHandle->Ibeta_est > INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ibeta_est = INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else if ( pHandle->Ibeta_est <= -INT16_MAX * ( int32_t )( pHandle->hF1 ) )
  {
    pHandle->Ibeta_est = -INT16_MAX * ( int32_t )( pHandle->hF1 );
  }
  else
  {
  }
//////////////////////////////////////////////////////////////	
//����Ia Ib �Ҳ��ƶ�15λ
#ifdef FULL_MISRA_C_COMPLIANCY
  hIalfa_err = ( int16_t )( pHandle->Ialfa_est / pHandle->hF1 );
#else
  hIalfa_err = ( int16_t )( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif
// ����Ia�͹���Ia�������
  hIalfa_err = hIalfa_err - pInputs->Ialfa_beta.alpha;

#ifdef FULL_MISRA_C_COMPLIANCY
  hIbeta_err = ( int16_t )( pHandle->Ibeta_est / pHandle->hF1 );
#else
  hIbeta_err = ( int16_t )( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif
// ����Ib�͹���Ib�������
  hIbeta_err = hIbeta_err - pInputs->Ialfa_beta.beta;
//?????????????????
  wAux = ( int32_t )( pInputs->Vbus ) * pInputs->Valfa_beta.alpha;//�������ߵ�ѹĿ���Ǽ�С���ߵ�ѹ������ϵͳ��Ӱ��
#ifdef FULL_MISRA_C_COMPLIANCY
  hValfa = ( int16_t ) ( wAux / 65536 );
#else
  hValfa = ( int16_t ) ( wAux >> 16 );
#endif

  wAux = ( int32_t )( pInputs->Vbus ) * pInputs->Valfa_beta.beta;
#ifdef FULL_MISRA_C_COMPLIANCY
  hVbeta = ( int16_t ) ( wAux / 65536 );
#else
  hVbeta = ( int16_t ) ( wAux >> 16 );
#endif

  /*alfa axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux = ( int16_t ) ( pHandle->Ialfa_est / pHandle->hF1 );
#else
  hAux = ( int16_t ) ( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif

//#define GAIN1                            -24129
//#define GAIN2                            19239
//#define C1 (int32_t)((((int16_t)F1)*RS)/(LS*TF_REGULATION_RATE))
//#define C2 (int32_t) GAIN1
//#define C3 (int32_t)((((int16_t)F1)*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))
//#define C4 (int32_t) GAIN2
//#define C5 (int32_t)((((int16_t)F1)*MAX_VOLTAGE)/(LS*MAX_CURRENT*TF_REGULATION_RATE))

/******************Ia����*********************************/
  //ia
  wAux = ( int32_t ) ( pHandle->hC1 ) * hAux;//Iest*Rs*T/Ls
  wIalfa_est_Next = pHandle->Ialfa_est - wAux;//Iest-Iest*Rs*T/Ls

  wAux = ( int32_t ) ( pHandle->hC2 ) * hIalfa_err;//K1T(Iest - Ialfa)
  wIalfa_est_Next += wAux;
	//���������Ϊʲô�������ߵ�ѹ �۲��ʱ����Բ���Ҳ���Եõ��Ƕ�
//wAux = ( int32_t )( pInputs->Vbus ) * pInputs->Valfa_beta.alpha;
//hValfa = ( int16_t ) ( wAux >> 16 );
  wAux = ( int32_t ) ( pHandle->hC5 ) * hValfa;//T*U(a)/Ls hC5 = (������ѹ*T)/(��������*Ls)
  wIalfa_est_Next += wAux;
//hAux_Alfa = ( int16_t )( pHandle->wBemf_alfa_est >> pHandle->F2LOG );//Ea�Ҳ��ƶ�12λ
  wAux = ( int32_t )  ( pHandle->hC3 ) * hAux_Alfa;//T*Ea/Ls hC3 = (��󷴵綯�Ƶ�ѹ*T)/(��������*Ls)
  wIalfa_est_Next -= wAux;
/******************END Ia����*********************************/
/******************���綯��Ua����*********************************/
  wAux = ( int32_t )( pHandle->hC4 ) * hIalfa_err;//(Iest - Ialfa)*K2T
  wBemf_alfa_est_Next = pHandle->wBemf_alfa_est + wAux;//Ea(��ǰ����ķ��綯��)+(Iest - Ialfa)*K2T

#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = ( int32_t ) hAux_Beta / pHandle->hF3;
#else
 //hAux_Beta = ( int16_t )( pHandle->wBemf_beta_est >> pHandle->F2LOG );//Ub�Ҳ��ƶ�12λ
  wAux = ( int32_t ) hAux_Beta >> pHandle->F3POW2;
#endif
//  wAux = ( int32_t )( pHandle->hF2 ) * pHandle->hF3;
//  pHandle->hC6 = ( int16_t )( wAux / C6_COMP_CONST2 );
  wAux = wAux * pHandle->hC6; //����STO_PLL_Init( STO_PLL_Handle_t * pHandle )�ó�
  wAux = pHandle->_Super.hElSpeedDpp * wAux;//�����Ƶ����� hElSpeedDpp*����*wBemf_beta_est û������Ϊʲô����Щ�������õı仯
  wBemf_alfa_est_Next += wAux;
/******************END ���綯�� a�����*********************************/
/******************���綯��b�����*********************************/
  /*beta axes observer*/
#ifdef FULL_MISRA_C_COMPLIANCY
  hAux = ( int16_t ) ( pHandle->Ibeta_est / pHandle->hF1 );
#else
  hAux = ( int16_t ) ( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif
  wAux = ( int32_t )  ( pHandle->hC1 ) * hAux;
  wIbeta_est_Next = pHandle->Ibeta_est - wAux;

  wAux = ( int32_t ) ( pHandle->hC2 ) * hIbeta_err;
  wIbeta_est_Next += wAux;

  wAux = ( int32_t ) ( pHandle->hC5 ) * hVbeta;
  wIbeta_est_Next += wAux;

  wAux = ( int32_t )  ( pHandle->hC3 ) * hAux_Beta;
  wIbeta_est_Next -= wAux;

  wAux = ( int32_t )( pHandle->hC4 ) * hIbeta_err;
  wBemf_beta_est_Next = pHandle->wBemf_beta_est + wAux;

#ifdef FULL_MISRA_C_COMPLIANCY
  wAux = ( int32_t )hAux_Alfa / pHandle->hF3;
#else
  wAux = ( int32_t ) hAux_Alfa >> pHandle->F3POW2;
#endif

  wAux = wAux * pHandle->hC6;
  wAux = pHandle->_Super.hElSpeedDpp * wAux;
  wBemf_beta_est_Next -= wAux;
/******************END ���綯�� b�����*********************************/
  if(pHandle->hForcedAvrSpeed_VSS >=0)//�ж�����ת
  {
    wDirection = 1;
  }
  else
  {
    wDirection = -1;
  }
//��ʼ���໷
  /*Calls the PLL blockset*/
  pHandle->hBemf_alfa_est = hAux_Alfa;//��ֵ���綯�ƹ���ֵ���綯��
  pHandle->hBemf_beta_est = hAux_Beta;

  hAux_Alfa = ( int16_t )( hAux_Alfa * wDirection );
  hAux_Beta = ( int16_t )( hAux_Beta * wDirection );
/***************************���໷���**********************************
��������໷ʵ���Ͼ���һ���ջ�ϵͳ��Ŀ�������һ���ȶ����ź�

����1Ϊʲô�������໷��ͼ tan��=Eb/Ea ����� �����������-sin��*Ea + cos��*Eb = 0ʱ ����ĽǶ���׼ȷ�Ļ��������С��
����2 ��ͼ�����Ƶ�
	���綯�ƹ�ʽ 
	Ea = ��m*P*Wr*cos(P*Wr*t)  -- (1)
	Eb = -��m*P*Wr*sin(P*Wr*t) -- (2)
  ���໷���㹫ʽ�ο�st�����ѵppt3 ��13ҳ ���໷��ͼ
	ͨ�����໷��ͼ�� -Ea*sin(��(k-1)) - Eb*cos(��(k-1)) ---��3��
	����ʽ 1 �� 2 ���루3��
	��A = ��m*P*Wr
	-A*cos(��(k))*sin(��(k-1)) + A*sin(��(k))*cos(��(k-1)) 
	ͨ�����ǹ�ʽ�� = A*sin����(k) - ��(k-1)��
	���޹�ʽ = A*(��(k) - ��(k-1)) --��4��
	����4��������0ʱ ���໷������ĽǶ�Խ������ʵֵ
***************************************************************************/

//���໷��ʽ -Ea*sin��thera��- Eb*cos��thera����ΪPI���������
//PI������� Ϊת��Wr�����ٶȣ� �������ֵõ�thera �Ƕ�
  hRotor_Speed = STO_ExecutePLL( pHandle, hAux_Alfa, -hAux_Beta );//ת��ת��
//˲ʱ�ٶ�ת�ӵ�
  pHandle->_Super.InstantaneousElSpeedDpp = hRotor_Speed;
//�洢ת���ٶ�
  STO_Store_Rotor_Speed( pHandle, hRotor_Speed );

  pHandle->_Super.hElAngle += hRotor_Speed;//�ٶȵĻ��־��ǽǶ�

  /*storing previous values of currents and bemfs*/
	//�������������һʱ���� ia ib ea eb ��ֵ����ǰʱ�̱��� ����һ��ִ��SMOʱʹ�� 
  pHandle->Ialfa_est = wIalfa_est_Next;
  pHandle->wBemf_alfa_est = wBemf_alfa_est_Next;

  pHandle->Ibeta_est = wIbeta_est_Next;
  pHandle->wBemf_beta_est = wBemf_beta_est_Next;

  return ( pHandle->_Super.hElAngle );//�õ������Ƕ�
}

/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and returns - through
  *         parameter hMecSpeedUnit - the rotor average mechanical speed,
  *         expressed in Unit. Average is computed considering a FIFO depth
  *         equal to bSpeedBufferSizeUnit. Moreover it also computes and returns
  *         the reliability state of the sensor.
  * @param  pHandle: handler of the current instance of the STO component
  * @param  pMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
  * @retval bool speed sensor reliability, measured with reference to parameters
  *         bReliability_hysteresys, hVariancePercentage and bSpeedBufferSize
  *         true = sensor information is reliable
  *         false = sensor information is not reliable
  */

__weak bool STO_PLL_CalcAvrgMecSpeedUnit( STO_PLL_Handle_t * pHandle, int16_t * pMecSpeedUnit )
{
  int32_t wAvrSpeed_dpp = ( int32_t )0;
  int32_t wError, wAux, wAvrSquareSpeed, wAvrQuadraticError = 0;
  uint8_t i, bSpeedBufferSizeUnit = pHandle->SpeedBufferSizeUnit;//���ٶȽṹ����˲������������ֵ
  int32_t wObsBemf, wEstBemf;
  int32_t wObsBemfSq = 0, wEstBemfSq = 0;
  int32_t wEstBemfSqLo;

  bool bIs_Speed_Reliable = false, bAux = false;
  bool bIs_Bemf_Consistent = false;
//��ֵ�˲�
  for ( i = 0u; i < bSpeedBufferSizeUnit; i++ )
  {
    wAvrSpeed_dpp += ( int32_t )( pHandle->Speed_Buffer[i] );
  }

  wAvrSpeed_dpp = wAvrSpeed_dpp / ( int16_t )bSpeedBufferSizeUnit;
//�����ٶȵķ��� ����ֵԽС֤���ٶ�Խ�ȶ�
  for ( i = 0u; i < bSpeedBufferSizeUnit; i++ )
  {
    wError = ( int32_t )( pHandle->Speed_Buffer[i] ) - wAvrSpeed_dpp;
    wError = ( wError * wError );
    wAvrQuadraticError += wError;
  }
  /*It computes the measurement variance   */
  wAvrQuadraticError = wAvrQuadraticError / ( int16_t )bSpeedBufferSizeUnit;

  /* The maximum variance acceptable is here calculated as a function of average
     speed                                                                    */
  wAvrSquareSpeed = wAvrSpeed_dpp * wAvrSpeed_dpp;//ƽ���ٶȵ�ƽ��
	//#define PERCENTAGE_FACTOR    (uint16_t)(VARIANCE_THRESHOLD*128u) 0.1*128
	//�ٶ����������̶� 
  wAvrSquareSpeed = ( wAvrSquareSpeed * ( int32_t )( pHandle->VariancePercentage )) / ( int16_t )128;
  //������С���ٶȲ��������̶ȵ�ʱ����Ϊ�ٶ���׼ȷ��
  if ( wAvrQuadraticError < wAvrSquareSpeed )// 
  {
    bIs_Speed_Reliable = true;
  }

  /*Computation of Mechanical speed Unit*/
	//���㵥λ��е�ٶ�
	//1s16degree = 2pi/65535
	//1ddp = {1/T(foc)} *  s16degree = {1/T(foc)} * {2pi/65535} 
	//#define TF_REGULATION_RATE_SCALED (uint16_t) ((uint32_t)(PWM_FREQUENCY)/(REGULATION_EXECUTION_RATE*PWM_FREQ_SCALING))
  // 16000/1*1
  wAux = wAvrSpeed_dpp * ( int32_t )( pHandle->_Super.hMeasurementFrequency );//ת��
  wAux = wAux * ( int32_t ) ( pHandle->_Super.SpeedUnit );//    *10->�Ŵ�10��
  wAux = wAux / ( int32_t )( pHandle->_Super.DPPConvFactor);//  360�ȶ�Ӧ65535  һ����ת����Ȧ��
  wAux = wAux / ( int16_t )( pHandle->_Super.bElToMecRatio );// /2(������)  ��еת��=����ת��/������

  *pMecSpeedUnit = ( int16_t )wAux;
  pHandle->_Super.hAvrMecSpeedUnit = ( int16_t )wAux;

  pHandle->IsSpeedReliable = bIs_Speed_Reliable;

  /*Bemf Consistency Check algorithm*/
  if ( pHandle->EnableDualCheck == true ) /*do algorithm if it's enabled*/
  {
    wAux = ( wAux < 0 ? ( -wAux ) : ( wAux ) ); /* wAux abs value   */
    if ( wAux < ( int32_t )( pHandle->MaxAppPositiveMecSpeedUnit ) )//((MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM)*1.15 //((4000*10)/60)*1.15
    {
      /*Computation of Observed back-emf*/
			//����۲����з��綯����ֵ  Ea^2 + Eb^2 = E^2
      wObsBemf = ( int32_t )( pHandle->hBemf_alfa_est );
      wObsBemfSq = wObsBemf * wObsBemf;
      wObsBemf = ( int32_t )( pHandle->hBemf_beta_est );
      wObsBemfSq += wObsBemf * wObsBemf;
			//����ķ��綯��
      /*Computation of Estimated back-emf*/
			//ͨ��ת�ټ���ĳ��ķ��綯�� E = Ce*n*��ͨ��
			//#define MAX_APPLICATION_SPEED_UNIT ((MAX_APPLICATION_SPEED_RPM*SPEED_UNIT)/_RPM)//4000*10/60*1.15(������ϵ��)
      wEstBemf = ( wAux * 32767 ) / ( int16_t )( pHandle->_Super.hMaxReliableMecSpeedUnit );
      wEstBemfSq = ( wEstBemf * ( int32_t )( pHandle->BemfConsistencyGain ) ) / 64;//64
      wEstBemfSq *= wEstBemf;
			//������ֵ
      /*Computation of threshold*/
      wEstBemfSqLo = wEstBemfSq -
                     ( wEstBemfSq / 64 ) * ( int32_t )( pHandle->BemfConsistencyCheck );//64

      /*Check*/
			//����ͨ���۲�ķ��綯�ƺ����໷������ٶȹ���ķ��綯��ȥ�ж��Ƿ��ת
      if ( wObsBemfSq > wEstBemfSqLo )
      {
        bIs_Bemf_Consistent = true;
      }
    }

    pHandle->IsBemfConsistent = bIs_Bemf_Consistent;
    pHandle->Obs_Bemf_Level = wObsBemfSq;
    pHandle->Est_Bemf_Level = wEstBemfSq;
  }
  else
  {
    bIs_Bemf_Consistent = true;
  }

  /*Decision making*/
  if ( pHandle->IsAlgorithmConverged == false )//�㷨������
  {
    bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );//�ж��ٶȵĿɿ���
  }
  else//����
  { //��⵽�ٶȲ����ϴ���߹���ķ��綯�ƺ͹۲����ķ��綯�Ʋ�����
    if ( ( pHandle->IsSpeedReliable == false ) || ( bIs_Bemf_Consistent == false ) )
    {
      pHandle->ReliabilityCounter++;
      if ( pHandle->ReliabilityCounter >= pHandle->Reliability_hysteresys )//�������������Ƿ����3
      {
        pHandle->ReliabilityCounter = 0u;
        pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
        bAux = false;
      }
      else
      {
        bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
      }
    }
    else
    {
      pHandle->ReliabilityCounter = 0u;
      bAux = SPD_IsMecSpeedReliable ( &pHandle->_Super, pMecSpeedUnit );
    }
  }
  return ( bAux );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed. It computes and update component
  *         variable hElSpeedDpp that is estimated average electrical speed
  *         expressed in dpp used for instance in observer equations.
  *         Average is computed considering a FIFO depth equal to
  *         bSpeedBufferSizedpp.
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */

//inline static void STO_Store_Rotor_Speed( STO_PLL_Handle_t * pHandle, int16_t hRotor_Speed )
//{

//  uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;//ÿִ��һ�ν��ϴ�ִ�еĴ�����ֵ���ֲ�����

//  bBuffer_index++;
//	//����ۼӵĴ��������ṹ�����鳤�����¹���
//  if ( bBuffer_index == pHandle->SpeedBufferSizeUnit )
//  {
//    bBuffer_index = 0u;
//  }
////��¼�ϴ��ٶ�
//  pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];
////�ѵ�ǰ�ٶȴ洢��������
//  pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
//  pHandle->Speed_Buffer_Index = bBuffer_index;//���ݵ�ǰ����λ�� �´�ִ�б�����ʱ�����ٶ�ֵ������һ�����������
//}
__weak void STO_PLL_CalcAvrgElSpeedDpp( STO_PLL_Handle_t * pHandle )
{

  int16_t hIndexNew = ( int16_t )pHandle->Speed_Buffer_Index;
  int16_t hIndexOld;
  int16_t hIndexOldTemp;
  int32_t wSum = pHandle->DppBufferSum;
  int32_t wAvrSpeed_dpp;
  int16_t hSpeedBufferSizedpp = ( int16_t )( pHandle->SpeedBufferSizeDpp );
  int16_t hSpeedBufferSizeUnit = ( int16_t )( pHandle->SpeedBufferSizeUnit );
  int16_t hBufferSizeDiff;

  hBufferSizeDiff = hSpeedBufferSizeUnit - hSpeedBufferSizedpp;

  if ( hBufferSizeDiff == 0 )
  {
		//ͨ��ÿһ�����ٶȵ��ٶȵ����������ٶ� 
		//ʵ�ֹ��� 
		//1.hIndexNew = 0ʱ SpeedBufferOldestEl = 0��
		//�״��ٶ�Speed_Buffer[hIndexNew]��Ϊ0, ��Speed_Buffer[hIndexNew] ��ֵ��wSum
		//2.hIndexNew = 1ʱ ��pHandle->Speed_Buffer[hIndexNew] -pHandle->SpeedBufferOldestEl��
		//Ϊ�ٶȵ��������ٶ�����������һ�ε��ٶ�wSum ��Ϊ��ǰ���ڵ��ٶ�
    wSum = wSum + pHandle->Speed_Buffer[hIndexNew] -
           pHandle->SpeedBufferOldestEl;
  }
  else
  {
    hIndexOldTemp = hIndexNew + hBufferSizeDiff;

    if ( hIndexOldTemp >= hSpeedBufferSizeUnit )
    {
      hIndexOld = hIndexOldTemp - hSpeedBufferSizeUnit;
    }
    else
    {
      hIndexOld = hIndexOldTemp;
    }

    wSum = wSum + pHandle->Speed_Buffer[hIndexNew] -
           pHandle->Speed_Buffer[hIndexOld];
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wAvrSpeed_dpp = wSum / hSpeedBufferSizeDpp;
#else
  wAvrSpeed_dpp = wSum >> pHandle->SpeedBufferSizeDppLOG;// ��64
#endif

  pHandle->_Super.hElSpeedDpp = ( int16_t )wAvrSpeed_dpp;
  pHandle->DppBufferSum = wSum;//��һ������ִ�б�����ʹ��
}

/**
  * @brief  It clears state observer component by re-initializing private variables
  * @param  pHandle related object of class CSTO_SPD
  * @retval none
  */
__weak void STO_PLL_Clear( STO_PLL_Handle_t * pHandle )
{
	//����ĵ���
  pHandle->Ialfa_est = ( int32_t )0;
  pHandle->Ibeta_est = ( int32_t )0;
	//����ķ��綯��
  pHandle->wBemf_alfa_est = ( int32_t )0;
  pHandle->wBemf_beta_est = ( int32_t )0;
	//�����Ƕ�
  pHandle->_Super.hElAngle = ( int16_t )0;
	//ÿִ��һ���㷨�õ��ĽǶ�
  pHandle->_Super.hElSpeedDpp = ( int16_t )0;
  pHandle->ConsistencyCounter = 0u;
  pHandle->ReliabilityCounter = 0u;
  pHandle->IsAlgorithmConverged = false;
  pHandle->IsBemfConsistent = false;
  pHandle->Obs_Bemf_Level = ( int32_t )0;
  pHandle->Est_Bemf_Level = ( int32_t )0;
  pHandle->DppBufferSum = ( int32_t )0;
  pHandle->ForceConvergency = false;
  pHandle->ForceConvergency2 = false;
//�����ٶ��˲�����������
  STO_InitSpeedBuffer( pHandle );
	//����������
  PID_SetIntegralTerm( & pHandle->PIRegulator, ( int32_t )0 );
}

/**
  * @brief  It stores in estimated speed FIFO latest calculated value of motor
  *         speed
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
inline static void STO_Store_Rotor_Speed( STO_PLL_Handle_t * pHandle, int16_t hRotor_Speed )
{

  uint8_t bBuffer_index = pHandle->Speed_Buffer_Index;//ÿִ��һ�ν��ϴ�ִ�еĴ�����ֵ���ֲ�����

  bBuffer_index++;
	//����ۼӵĴ��������ṹ�����鳤�����¹���
  if ( bBuffer_index == pHandle->SpeedBufferSizeUnit )
  {
    bBuffer_index = 0u;
  }
//��¼�ϴ��ٶ�
  pHandle->SpeedBufferOldestEl = pHandle->Speed_Buffer[bBuffer_index];
//�ѵ�ǰ�ٶȴ洢��������
  pHandle->Speed_Buffer[bBuffer_index] = hRotor_Speed;
  pHandle->Speed_Buffer_Index = bBuffer_index;//���ݵ�ǰ����λ�� �´�ִ�б�����ʱ�����ٶ�ֵ������һ�����������
}

/**
  * @brief  It executes PLL algorithm for rotor position extraction from B-emf
  *         alpha and beta
  * @param  pHandle: handler of the current instance of the STO component
  *         hBemf_alfa_est estimated Bemf alpha on the stator reference frame
  *         hBemf_beta_est estimated Bemf beta on the stator reference frame
  * @retval none
  */
/***************************���໷���**********************************
��������໷ʵ���Ͼ���һ���ջ�ϵͳ��Ŀ�������һ���ȶ����ź�

����1Ϊʲô�������໷��ͼ tan��=Eb/Ea ����� �����������sin��*Ea - cos��*Eb = 0 ������¼���ĽǶ��ǶԵĻ�����������С��
����2 ��ͼ�����Ƶ�
	���綯�ƹ�ʽ 
	Ea = ��m*P*Wr*cos(P*Wr*t)  -- (1)
	Eb = -��m*P*Wr*sin(P*Wr*t) -- (2)
  ���໷���㹫ʽ�ο�st�����ѵppt3 ��13ҳ ���໷��ͼ
	ͨ�����໷��ͼ�� -Ea*sin(��(k-1)) - Eb*cos(��(k-1)) ---��3��
	����ʽ 1 �� 2 ���루3��
	��A = ��m*P*Wr
	-A*cos(��(k))*sin(��(k-1)) + A*sin(��(k))*cos(��(k-1)) 
	ͨ�����ǹ�ʽ�� = A*sin����(k) - ��(k-1)��
	���޹�ʽ = A*(��(k) - ��(k-1)) --��4��
	����4��������0ʱ ���໷������ĽǶ�Խ������ʵֵ
***************************************************************************/
inline static int16_t STO_ExecutePLL( STO_PLL_Handle_t * pHandle, int16_t hBemf_alfa_est, int16_t
                               hBemf_beta_est )
{
  int32_t wAlfa_Sin_tmp, wBeta_Cos_tmp;
  int16_t hOutput;
  Trig_Components Local_Components;
  int16_t hAux1, hAux2;

  Local_Components = MCM_Trig_Functions( pHandle->_Super.hElAngle );

  /* Alfa & Beta BEMF multiplied by Cos & Sin*/
	//����ע�͹�ʽ3�е�����
  wAlfa_Sin_tmp = ( int32_t )( hBemf_alfa_est ) * ( int32_t )Local_Components.hSin;
  wBeta_Cos_tmp = ( int32_t )( hBemf_beta_est ) * ( int32_t )Local_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  hAux1 = ( int16_t )( wBeta_Cos_tmp / 32768 );
#else
  hAux1 = ( int16_t )( wBeta_Cos_tmp >> 15 );//q15 ��ʽ
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
  hAux2 = ( int16_t )( wAlfa_Sin_tmp / 32768 );
#else
  hAux2 = ( int16_t )( wAlfa_Sin_tmp >> 15 );
#endif
//PI ����
  /* Speed PI regulator */
  hOutput = PI_Controller( & pHandle->PIRegulator, ( int32_t )( hAux1 ) - hAux2 );

  return ( hOutput );
}

/**
  * @brief  It clears the estimated speed buffer
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
static void STO_InitSpeedBuffer( STO_PLL_Handle_t * pHandle )
{
  uint8_t b_i;
  uint8_t bSpeedBufferSize = pHandle->SpeedBufferSizeUnit;

  /*init speed buffer*/
  for ( b_i = 0u; b_i < bSpeedBufferSize; b_i++ )
  {
    pHandle->Speed_Buffer[b_i] = ( int16_t )0;
  }
  pHandle->Speed_Buffer_Index = 0u;
  pHandle->SpeedBufferOldestEl = ( int16_t )0;

  return;
}

/**
  * @brief  It internally performs a set of checks necessary to state whether
  *         the state observer algorithm converged. To be periodically called
  *         during motor open-loop ramp-up (e.g. at the same frequency of
  *         SPD_CalcElAngle), it returns true if the estimated angle and speed
  *         can be considered reliable, false otherwise
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hForcedMecSpeedUnit Mechanical speed in 0.1Hz unit as forced by VSS
  * @retval bool sensor reliability state
  */
__weak bool STO_PLL_IsObserverConverged( STO_PLL_Handle_t * pHandle, int16_t hForcedMecSpeedUnit )
{
  int16_t hEstimatedSpeedUnit, hUpperThreshold, hLowerThreshold;
  int32_t wAux;
  bool bAux = false;
  int32_t wtemp;

  
  pHandle->hForcedAvrSpeed_VSS = hForcedMecSpeedUnit;
 //�������������Ҫ
  if ( pHandle->ForceConvergency2 == true )
  {
    hForcedMecSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;
  }

  if ( pHandle->ForceConvergency == true )
  {
    bAux = true;
    pHandle->IsAlgorithmConverged = true;
    pHandle->_Super.bSpeedErrorNumber = 0u;
  }
  else //��������
  {
    hEstimatedSpeedUnit = pHandle->_Super.hAvrMecSpeedUnit;//STO_PLL_CalcAvrgMecSpeedUnit ��ͨ��sto������ٶ�

    wtemp = ( int32_t )hEstimatedSpeedUnit * ( int32_t )hForcedMecSpeedUnit;
//�������ٶȷ��ź�ǿ���������ٶȷ��ű�Ϊͬ��
    if ( wtemp > 0 )
    {
      if ( hEstimatedSpeedUnit < 0 )
      {
        hEstimatedSpeedUnit = -hEstimatedSpeedUnit;
      }

      if ( hForcedMecSpeedUnit < 0 )
      {
        hForcedMecSpeedUnit = -hForcedMecSpeedUnit;
      }
			//ǿ���������ٶȱ��������ֵ
      wAux = ( int32_t ) ( hForcedMecSpeedUnit ) * ( int16_t )pHandle->SpeedValidationBand_H;
      hUpperThreshold = ( int16_t )( wAux / ( int32_t )16 );

      wAux = ( int32_t ) ( hForcedMecSpeedUnit ) * ( int16_t )pHandle->SpeedValidationBand_L;
      hLowerThreshold = ( int16_t )( wAux / ( int32_t )16 );

      /* If the variance of the estimated speed is low enough...*/

//	.VariancePercentage                 =	PERCENTAGE_FACTOR,
// .SpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT,
// .SpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT,
// .MinStartUpValidSpeed               =	OBS_MINIMUM_SPEED_UNIT,
// .StartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,
      if ( pHandle->IsSpeedReliable == true )			//�ٶȲ����㹻С �ٶȵķ������Ҫ��
      {
				//����Ҫ����ٶȷ�Χ
        if ( ( uint16_t )hEstimatedSpeedUnit > pHandle->MinStartUpValidSpeed )
        {
          /*...and the estimated value is quite close to the expected value... */
          if ( hEstimatedSpeedUnit >= hLowerThreshold )
          {
            if ( hEstimatedSpeedUnit <= hUpperThreshold )
            {
              pHandle->ConsistencyCounter++;//�ȶ��Լ����ۼ�

              /*... for hConsistencyThreshold consecutive times... */
              if ( pHandle->ConsistencyCounter >=
                   pHandle->StartUpConsistThreshold )//�ȶ��Ĵ�������Ҫ�� ֤������
              {

                /* the algorithm converged.*/
                bAux = true;
                pHandle->IsAlgorithmConverged = true;
                pHandle->_Super.bSpeedErrorNumber = 0u;
              }
            }
            else
            {
              pHandle->ConsistencyCounter = 0u;
            }
          }
          else
          {
            pHandle->ConsistencyCounter = 0u;
          }
        }
        else
        {
          pHandle->ConsistencyCounter = 0u;
        }
      }
      else
      {
        pHandle->ConsistencyCounter = 0u;
      }
    }
  }

  return ( bAux );
}

/**
  * @brief  It exports estimated Bemf alpha-beta in alphabeta_t format
  * @param  pHandle: handler of the current instance of the STO component
  * @retval alphabeta_t Bemf alpha-beta
  */
__weak alphabeta_t STO_PLL_GetEstimatedBemf( STO_PLL_Handle_t * pHandle )
{
  alphabeta_t Vaux;
  Vaux.alpha = pHandle->hBemf_alfa_est;
  Vaux.beta = pHandle->hBemf_beta_est;
  return ( Vaux );
}


/**
  * @brief  It exports the stator current alpha-beta as estimated by state
  *         observer
  * @param  pHandle: handler of the current instance of the STO component
  * @retval alphabeta_t State observer estimated stator current Ialpha-beta
  */
__weak alphabeta_t STO_PLL_GetEstimatedCurrent( STO_PLL_Handle_t * pHandle )
{
  alphabeta_t Iaux;

#ifdef FULL_MISRA_C_COMPLIANCY
  Iaux.alpha = ( int16_t )( pHandle->Ialfa_est / ( pHandle->hF1 ) );
#else
  Iaux.alpha = ( int16_t )( pHandle->Ialfa_est >> pHandle->F1LOG );
#endif

#ifdef FULL_MISRA_C_COMPLIANCY
  Iaux.beta = ( int16_t )( pHandle->Ibeta_est / ( pHandle->hF1 ) );
#else
  Iaux.beta = ( int16_t )( pHandle->Ibeta_est >> pHandle->F1LOG );
#endif

  return ( Iaux );
}

/**
  * @brief  It exports current observer gains through parameters hhC2 and hhC4
  * @param  pHandle: handler of the current instance of the STO component
  * @param  phC2 pointer to int16_t used to return parameters hhC2
  * @param  phC4 pointer to int16_t used to return parameters hhC4
  * @retval none
  */
__weak void STO_PLL_GetObserverGains( STO_PLL_Handle_t * pHandle, int16_t * phC2, int16_t * phC4 )
{
  *phC2 = pHandle->hC2;
  *phC4 = pHandle->hC4;
}


/**
  * @brief  It allows setting new values for observer gains
  * @param  pHandle: handler of the current instance of the STO component
  * @param  wK1 new value for observer gain hhC1
  * @param  wK2 new value for observer gain hhC2
  * @retval none
  */
__weak void STO_PLL_SetObserverGains( STO_PLL_Handle_t * pHandle, int16_t hhC1, int16_t hhC2 )
{

  pHandle->hC2 = hhC1;
  pHandle->hC4 = hhC2;
}

/**
  * @brief  It exports current PLL gains through parameters pPgain and pIgain
  * @param  pHandle: handler of the current instance of the STO component
  * @param  pPgain pointer to int16_t used to return PLL proportional gain
  * @param  pIgain pointer to int16_t used to return PLL integral gain
  * @retval none
  */
__weak void STO_GetPLLGains( STO_PLL_Handle_t * pHandle, int16_t * pPgain, int16_t * pIgain )
{

  *pPgain = PID_GetKP( & pHandle->PIRegulator );
  *pIgain = PID_GetKI( & pHandle->PIRegulator );
}


/**
  * @brief  It allows setting new values for PLL gains
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hPgain new value for PLL proportional gain
  * @param  hIgain new value for PLL integral gain
  * @retval none
  */
__weak void STO_SetPLLGains( STO_PLL_Handle_t * pHandle, int16_t hPgain, int16_t hIgain )
{
  PID_SetKP( & pHandle->PIRegulator, hPgain );
  PID_SetKI( & pHandle->PIRegulator, hIgain );
}


/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle.
  *         Note: Mechanical angle management is not implemented in this
  *         version of State observer sensor class.
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
__weak void STO_PLL_SetMecAngle( STO_PLL_Handle_t * pHandle, int16_t hMecAngle )
{
}

/**
  * @brief  It resets integral term of PLL during on-the-fly startup
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
__weak void STO_OTF_ResetPLL( STO_Handle_t * pHandle )
{
  STO_PLL_Handle_t * pHdl = ( STO_PLL_Handle_t * )pHandle->_Super;
  PID_SetIntegralTerm( &pHdl->PIRegulator, ( int32_t )0 );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It resets integral term of PLL
  * @param  pHandle: handler of the current instance of the STO component
  * @retval none
  */
__weak void STO_ResetPLL( STO_PLL_Handle_t * pHandle )
{
  PID_SetIntegralTerm( &pHandle->PIRegulator, ( int32_t )0 );
}

/**
  * @brief  It sends locking info for PLL
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hElSpeedDpp:
  * @param  hElAngle:
  * @retval none
  */
__weak void STO_SetPLL( STO_PLL_Handle_t * pHandle, int16_t hElSpeedDpp, int16_t hElAngle )
{
  PID_SetIntegralTerm( & pHandle->PIRegulator,
                       ( int32_t )hElSpeedDpp * ( int32_t )PID_GetKIDivisor( & pHandle->PIRegulator ) );
  pHandle->_Super.hElAngle = hElAngle;
}

/**
  * @brief  It exports estimated Bemf squared level
  * @param  pHandle: handler of the current instance of the STO component
  * @retval int32_t
  */
__weak int32_t STO_PLL_GetEstimatedBemfLevel( STO_PLL_Handle_t * pHandle )
{
  return ( pHandle->Est_Bemf_Level );
}

/**
  * @brief  It exports observed Bemf squared level
  * @param  pHandle: handler of the current instance of the STO component
  * @retval int32_t
  */
__weak int32_t STO_PLL_GetObservedBemfLevel( STO_PLL_Handle_t * pHandle )
{
  return ( pHandle->Obs_Bemf_Level );
}

/**
  * @brief  It enables/disables the bemf consistency check
  * @param  pHandle: handler of the current instance of the STO component
  * @param  bSel boolean; true enables check; false disables check
  */
__weak void STO_PLL_BemfConsistencyCheckSwitch( STO_PLL_Handle_t * pHandle, bool bSel )
{
  pHandle->EnableDualCheck = bSel;
}

/**
  * @brief  It returns the result of the Bemf consistency check
  * @param  pHandle: handler of the current instance of the STO component
  * @retval bool Bemf consistency state
  */
__weak bool STO_PLL_IsBemfConsistent( STO_PLL_Handle_t * pHandle )
{
  return ( pHandle->IsBemfConsistent );
}

/**
  * @brief  It returns the result of the last variance check
  * @param  pHandle: handler of the current instance of the STO component
  * @retval bool Variance state
  */
__weak bool STO_PLL_IsVarianceTight( const STO_Handle_t * pHandle )
{
  STO_PLL_Handle_t * pHdl = ( STO_PLL_Handle_t * )pHandle->_Super;
  return ( pHdl->IsSpeedReliable );
}

/**
  * @brief  It forces the state-observer to declare convergency
  * @param  pHandle: handler of the current instance of the STO component
  */
__weak void STO_PLL_ForceConvergency1( STO_Handle_t * pHandle )
{
  STO_PLL_Handle_t * pHdl = ( STO_PLL_Handle_t * )pHandle->_Super;
  pHdl->ForceConvergency = true;
}

/**
  * @brief  It forces the state-observer to declare convergency
  * @param  pHandle: handler of the current instance of the STO component
  */
__weak void STO_PLL_ForceConvergency2( STO_Handle_t * pHandle )
{
  STO_PLL_Handle_t * pHdl = ( STO_PLL_Handle_t * )pHandle->_Super;
  pHdl->ForceConvergency2 = true;
}

/**
  * @brief  Set the Absolute value of minimum mechanical speed (expressed in
  *         the unit defined by #SPEED_UNIT) required to validate the start-up.
  * @param  pHandle: handler of the current instance of the STO component
  * @param  hMinStartUpValidSpeed: Absolute value of minimum mechanical speed
  */
__weak void STO_SetMinStartUpValidSpeedUnit( STO_PLL_Handle_t * pHandle, uint16_t hMinStartUpValidSpeed )
{
  pHandle->MinStartUpValidSpeed = hMinStartUpValidSpeed;
}

/**
  * @}
  */

/**
  * @}
  */

/** @} */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
