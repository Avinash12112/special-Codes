/** @file		aAccDec.h
 *  @brief		An initial aAccDec.h
 *
 *				This file contains the acceleration and deceleration functions parameters.
 *
 *  @author		DD RK_avinash
 *
 *  @date		12-Nov-2020
 */

#ifndef AACCDEC_H_
#define AACCDEC_H_
#include "VDM_STDINC.H"

/*used structure to use ramp function for different  purposes (not only for speed)*/
typedef struct
{
	INT32 ramp_up_count;
	INT32 ramp_down_count;
	INT32 ramp_up_inc;
	INT32 ramp_dn_dec;
	INT16 ramp_val;		/*Defined in structure to initialize the value before calling init function*/
}T_RAMP_UP_DOWN;

/*Enum for explaining the status of acc_dec initialisation*/
typedef enum
{
	ACC_DEC_SUCCESS,
	ACC_DEC_INIT_ERROR,
	ACC_DEC_HIGH_SAMPLING_TIME,
}ACC_DEC_STATUS;

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
ACC_DEC_STATUS aAccDecinit(FLOAT acc_time_in_Sec, FLOAT dec_time_in_Sec, FLOAT sampling_Time, T_RAMP_UP_DOWN *psparam);

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
ACC_DEC_STATUS aAccDecvalsec_init(UINT16 max_op_value,UINT16 acc_valpSec,UINT16 dec_valpSec,FLOAT sampling_Time,T_RAMP_UP_DOWN *psparam);

/** @name 		aAccDec(INT16 target, T_RAMP_UP_DOWN *psparam);
 *  @brief 		to do acc_dec on each sampling time.
 *
 *  @param
 *  					target			- 	Target speed,
 *  					T_RAMP_UP_DOWN	-	Structure to pass the count value required to reach target speed,
 *
 *  @return 	INT16 returns the current speed reference.
 */
INT16 aAccDec(INT16 target, T_RAMP_UP_DOWN *psparam);

#endif /* AACCDEC_H_ */
