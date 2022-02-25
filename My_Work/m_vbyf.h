/*
 * m_vbyf.h
 *
 *  Created on: 11-Nov-2020
 *      Author: dd_avinash
 */

#ifndef M_VBYF_H_
#define M_VBYF_H_
#include "mlib.h"
#include "gflib.h"
#include "gmclib.h"
#include "gdflib.h"
#include "amclib.h"
#include "VDM_STDINC.H"

#define __PI__				(3.141592654)
//#define UMAX				400				// maximum DC bus voltage measured in ADC
//#define DC_BUS_COMPENSATION 0

/**
 *	vbyf_parameters to get the parameters from user
 */

typedef struct
{
	INT16 maximum_op_frequency;
	INT16 rated_voltage;
	INT16 rated_frequency;
	INT16 middle_op_frequency;
	INT16 minimum_op_frequency;
	INT16 minimum_op_voltage;
	INT16 switching_frequency;
	INT16 DC_Bus_maximum;
}vbyf_parameters;

/**
 *	vbyf_return to store angle and magnitude
 */

typedef struct
{
	INT16 theta;
	INT16 uOut_f16;

}vbyf_return;

/**
 *	voltage_conversion to do dq to svm inverse transformation parameters.
 */

typedef struct
{
	GMCLIB_2COOR_DQ_T_F16 sUDQReq; 	/* Required DQ voltage */
	INT16 UDcBusFilt;			/* if DC Bus compensation has to be used, Pass Measured DC Bus voltage in 16 bit value,
										else set the value as 32767*/
	INT16 position;
	GMCLIB_2COOR_SINCOS_T_F16 sAnglePosEl;
}voltage_conversion;

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
void Vbyf_init(vbyf_parameters psset);

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
vbyf_return do_vbyf(INT16 Freq_ref);/*called on each sampling time*/

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
voltage_conversion vf_dqConversion(vbyf_return vf);

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
GMCLIB_3COOR_T_F16 dq_toSVM(voltage_conversion dq_SVM, UINT16 *sector);

#endif /* M_VBYF_H_ */
