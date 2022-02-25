/** @file		aAccDec.c
 *  @brief		An initial aAccDec.c
 *
 *				This file contains the acceleration and deceleration functions parameters.
 *
 *  @author		DD RK_avinash
 *
 *  @date		12-Nov-2020
 */

#include "aAccDec.h"

/** @name 		aAccDecinit(FLOAT acc_time_in_Sec, FLOAT dec_time_in_Sec,
 * 										FLOAT sampling_Time,
 * 												 T_RAMP_UP_DOWN *psparam);
 *
 *  @brief 		to calculate the constants required to do acc_dec.
 *
 *  @param 				acc_time_in_Sec		-	acceleration time to reach from zero to maximum value,
 *  					dec_time_in_Sec		-	deceleration time to reach from maximum value to zero,
 *  					sampling_Time		- 	Function calling time or sampling time to do ACC_DEC,
 *  					T_RAMP_UP_DOWN		-	Structure to initialize the count required to reach target speed,
 *
 *  @return 	void.
 *
 *  @warning	sampling time should be less than acc_time_in_Sec and dec_time_in_Sec.
 */

ACC_DEC_STATUS aAccDecinit(FLOAT acc_time_in_Sec, FLOAT dec_time_in_Sec, FLOAT sampling_Time, T_RAMP_UP_DOWN *psparam)
{
	if(sampling_Time>0.01)
	{
		return ACC_DEC_HIGH_SAMPLING_TIME;
	}

	/*calculating the up count required to reach the from zero to maximum speed*/
	psparam->ramp_up_count = (((FLOAT)acc_time_in_Sec)/(sampling_Time));

	/*calculating the down count required to reach the from maximum speed to zero*/
	psparam->ramp_down_count = (((FLOAT)dec_time_in_Sec)/(sampling_Time));

	if((psparam->ramp_up_count<=0)||(psparam->ramp_down_count<=0))
	{
		return ACC_DEC_INIT_ERROR;
	}

	/*setting the up count and down count value based on initial ramp_val assigned before calling this function*/
	psparam->ramp_up_inc = (((INT64)psparam->ramp_val)*psparam->ramp_up_count)/(32767);
	psparam->ramp_dn_dec = (((INT64)psparam->ramp_val)*psparam->ramp_down_count)/(32767);

	return ACC_DEC_SUCCESS;
}

/** @name 		aAccDecvalsec_init(INT16 max_op_value,INT16 acc_valpSec,
 * 														INT16 dec_valpSec,FLOAT sampling_Time,
 * 																				T_RAMP_UP_DOWN *psparam);
 *  @brief 		to calculate the constants required to do acc_dec.
 *
 *  @param 		max_op_value	-	maximum output value,
 *  					acc_valpSec		-	accvalue per sec,
 *  					dec_valpSec		-	Decvalue per sec,
 *  					sampling_Time	- 	Function calling time or sampling time to do ACC_DEC,
 *  					T_RAMP_UP_DOWN	-	Structure to initialize the count required to reach target speed,
 *
 *  @return 	void.
 */
ACC_DEC_STATUS aAccDecvalsec_init(UINT16 max_op_value,
								UINT16 acc_valpSec,
								UINT16 dec_valpSec,
								FLOAT sampling_Time,	// function calling time or sampling time
								T_RAMP_UP_DOWN *psparam)
{
	if(sampling_Time>0.01)
	{
		return ACC_DEC_HIGH_SAMPLING_TIME;
	}

	/*calculating the up count required to reach the maximum speed*/
	psparam->ramp_up_count = (((FLOAT)max_op_value)/(acc_valpSec*sampling_Time));

	/*calculating the down count required to reach the maximum speed*/
	psparam->ramp_down_count = (((FLOAT)max_op_value)/(dec_valpSec*sampling_Time));

	if((psparam->ramp_up_count<=0)||(psparam->ramp_down_count<=0))
	{
		return ACC_DEC_INIT_ERROR;
	}

	/*setting the up count and down count value to zero*/
	/*setting the up count and down count value based on initial ramp_val assigned before calling this function*/
	psparam->ramp_up_inc = (((INT64)psparam->ramp_val)*psparam->ramp_up_count)/(32767);
	psparam->ramp_dn_dec = (((INT64)psparam->ramp_val)*psparam->ramp_down_count)/(32767);

	return ACC_DEC_SUCCESS;
}
/** @name 		aAccDec(INT16 target, T_RAMP_UP_DOWN *psparam);
 *  @brief 		to do acc_dec on each sampling time.
 *
 *  @param
 *  					target			- 	Target speed,
 *  					T_RAMP_UP_DOWN	-	Structure to pass the count value required to reach target speed,
 *
 *  @return 	INT16 returns the current speed reference.
 */

INT16 aAccDec(INT16 target, T_RAMP_UP_DOWN *psparam)
{
	/*Frequency ramp up*/
	if (psparam->ramp_val < target)
	{
		/*count increment*/
		psparam->ramp_up_inc++;

		/*calculating the current speed based on the count we received*/
		psparam->ramp_val = ((INT64)psparam->ramp_up_inc*32767)/psparam->ramp_up_count;

		if (psparam->ramp_val >= target)
		{
			psparam->ramp_val = target;
		}

		/*simultaneously counting the ramp down value for sudden change in target value less than current value*/
		psparam->ramp_dn_dec = (((INT64)psparam->ramp_val)*psparam->ramp_down_count)/(32767);
	}

	/*Frequency ramp down*/
	else if (psparam->ramp_val> target)
	{
		/*count decrement*/
		psparam->ramp_dn_dec--;

		/*calculating the current speed based on the count we received*/
		psparam->ramp_val = ((INT64)psparam->ramp_dn_dec*32767)/psparam->ramp_down_count;

		if (psparam->ramp_val <= target)
		{
			psparam->ramp_val = target;
		}

		/*simultaneously counting the ramp up value for sudden change in target value greater than current value*/
		psparam->ramp_up_inc = (((INT64)psparam->ramp_val)*psparam->ramp_up_count)/(32767);
	}

	return (psparam->ramp_val);
}
