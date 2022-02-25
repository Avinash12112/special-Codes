/*
 * m_vbyf.c
 *
 *  Created on: 11-Nov-2020
 *      Author: dd_avinash
 */
#include "m_vbyf.h"

typedef struct
{
	INT16 f16_minFreq;
	INT16 f16_midFreq;
	INT16 vbyf_constant;
	INT16 vbyf_rated;
	INT16 theta_intgra;
	INT16 minimum_op_voltage;
}vbyf_controlvalues;

LOCAL vbyf_controlvalues m_psctrl;
LOCAL INT16 m_f16_rated_voltage;

/** @name 		Vbyf_init(vbyf_parameters psset).
 *
 *	@brief		Initialize values required to calculate the v/f.
 *
 *  @param 		psset		vbyf_parameters.
 *
 *  @return 	none.
 *
 * 	@warning	none.
 */
void Vbyf_init(vbyf_parameters psset)
{
	float middle_op_voltage;
	INT16 l_max_freq_radsec_pi;

	l_max_freq_radsec_pi = 2*(psset.maximum_op_frequency);

	/*To get the theta constants referred from AN4642*//*Divided by 10 to convert per unit value*/
	m_psctrl.theta_intgra = ((((INT32)l_max_freq_radsec_pi*32767))/(psset.switching_frequency*10));

	/*vbyf_constant above middle frequency, conversion is multiplied by 4096 to increase constant scale*/
	m_psctrl.vbyf_rated = ((INT64)psset.rated_voltage*1414*(psset.maximum_op_frequency)<<12)/((psset.rated_frequency)*psset.DC_Bus_maximum*1000);

	/*calculating middle voltage */
	middle_op_voltage = ((float)psset.rated_voltage*(psset.middle_op_frequency))/psset.rated_frequency;

	/*vbyf_constant below middle frequency, conversion is multiplied by 4096 to increase constant scale*/
	m_psctrl.vbyf_constant = (((float)(middle_op_voltage - psset.minimum_op_voltage)*1414*(psset.maximum_op_frequency))*4096)/(((psset.middle_op_frequency) - (psset.minimum_op_frequency))*psset.DC_Bus_maximum*1000);

	/*initializing Minimum frequency to frac16 value*/
	m_psctrl.f16_minFreq = (((INT32)psset.minimum_op_frequency<<15)/(psset.maximum_op_frequency));

	/*initializing Middle frequency to frac16 value*/
	m_psctrl.f16_midFreq = (((INT32)psset.middle_op_frequency<<15)/(psset.maximum_op_frequency));

	/*passing minimum_op_voltage to control values to operate below middle frequency in frac16*/
	m_psctrl.minimum_op_voltage = ((INT32)psset.minimum_op_voltage*1414<<15)/(psset.DC_Bus_maximum*1000);

	/*Rated voltage in frac16*/
	m_f16_rated_voltage = ((float)psset.rated_voltage*1.414*32767)/psset.DC_Bus_maximum;
}

/** @name 		do_vbyf(frac16_t Freq_ref).
 *
 *	@brief		function to do v/f on sampling time.
 *
 *  @param 		Freq_ref	frequency reference.
 *
 *  @return 	vbyf_return with theta and magnitude.
 *
 * 	@warning	none.
 */
vbyf_return do_vbyf(INT16 Freq_ref)														/*called on each sampling time*/
{
	static vbyf_return vf;
	/*Refer the curve taught by Durga sir*/
	if(MLIB_Abs_F16_Ci(Freq_ref)<=m_psctrl.f16_midFreq)
		{
			vf.uOut_f16 = (((INT32)(MLIB_Abs_F16_Ci(Freq_ref)-m_psctrl.f16_minFreq)*m_psctrl.vbyf_constant)>>12) + (m_psctrl.minimum_op_voltage);
		}
	else if(MLIB_Abs_F16_Ci(Freq_ref)>m_psctrl.f16_midFreq)
		{
			vf.uOut_f16 = ((INT32)MLIB_Abs_F16_Ci(Freq_ref)*m_psctrl.vbyf_rated)>>12;
		}

	/*Limit the output voltage*/
	if(vf.uOut_f16>m_f16_rated_voltage)
	{
		vf.uOut_f16 = m_f16_rated_voltage;
	}

	/*To calculate the theta*/
		vf.theta +=  (Freq_ref*m_psctrl.theta_intgra)>>15;

	return vf;
}

/** @name 		voltage_conversion vf_dqConversion(vbyf_return vf).
 *
 *	@brief		function to initialize the parameters to do dq to svm conversion.
 *
 *  @param 		vf		vbyf_return.
 *
 *  @return 	voltage_conversion with parameters todo dq to svm conversion.
 *
 * 	@warning	none.
 */
voltage_conversion vf_dqConversion(vbyf_return vf)
{
	voltage_conversion Vbyf;

	Vbyf.position = vf.theta;
	Vbyf.sUDQReq.f16D = 0;
	Vbyf.sUDQReq.f16Q = vf.uOut_f16;

#ifdef DC_BUS_COMPENSATION
	Vbyf.UDcBusFilt = 32767; /*analog BUS voltage*/
#else
	Vbyf.UDcBusFilt = 32767;
#endif

	return Vbyf;
	/**************************************/
}

/** @name 		dq_toSVM(voltage_conversion dq_SVM).
 *
 *	@brief		dq to SVM transformation, returning duty values.
 *
 *  @param 		dq_SVM		voltage_conversion.
 *
 *  @return 	GMCLIB_3COOR_T_F16.
 *
 * 	@warning	none.
 */
GMCLIB_3COOR_T_F16 dq_toSVM(voltage_conversion dq_SVM, UINT16 *sector)
{
	GMCLIB_2COOR_ALBE_T_F16 sUAlBeReq,sUAlBecomp;        			/* Required Alpha/Beta voltage */
	GMCLIB_3COOR_T_F16 sDutyABC;
	GMCLIB_ParkInv_F16(&dq_SVM.sUDQReq,&dq_SVM.sAnglePosEl,&sUAlBeReq);

	/*DC BUS compensation is compulsory for V/f method*/
	GMCLIB_ElimDcBusRipFOC_F16(dq_SVM.UDcBusFilt,&sUAlBeReq,&sUAlBecomp);	// Do DC bus compensation
	/*Make sure to add DCBUS compensation before calling SVPWM*/
	*sector = GMCLIB_SvmStd_F16(&sUAlBeReq,&sDutyABC);
	return sDutyABC;
}
