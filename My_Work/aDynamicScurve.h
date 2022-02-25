/*
 * aScurve.h
 *
 *  Created on: 15-May-2021
 *      Author: dd.avinash
 */

#ifndef ADYNAMICSCURVE_H_
#define ADYNAMICSCURVE_H_

#include "VDM_STDINC.H"

typedef struct
{
	INT16 Ramp_up_sec;					// val_per_sec
	INT16 Ramp_dn_sec;					// val_per_sec
	INT16 Acc_start;					// in milli_seconds
	INT16 Acc_end;						// in milli_seconds
	INT16 Dec_start;					// in milli_seconds
	INT16 Dec_end;						// in milli_seconds
	FLOAT sampling_time;				// seconds

}T_SCURVE_RAMP_INPUT;

typedef struct
{
	T_SCURVE_RAMP_INPUT SCurve_input;
	FLOAT total_acc_time;				//Total acceleration time profile
	FLOAT total_accn_time;				//Total normal acceleration time profile
	FLOAT total_decn_time;				//Total normal deceleration time profile
	FLOAT total_dec_time;				//Total deceleration time profile
	FLOAT act_ramp_up_sec;				//Actual ramp up value
	FLOAT act_ramp_dn_sec;				//Actual ramp down value
	FLOAT prev_target_val;				//Previous target value
	FLOAT acc_astr;						//acceleration at start
	FLOAT acc_aend;						//acceleration at end
	FLOAT dec_astr;						//deceleration at start
	FLOAT dec_aend;						//deceleration at end
	FLOAT accleration_ramp_profile;		//acceleration ramp profile
	FLOAT output_val;					//acceleration ramp profile
	UINT32 counter;						//Total counter value to reach the target value
}T_SCURVE_RAMP;

VOID aDynamicScurve_init(T_SCURVE_RAMP *l_S_ramp,T_SCURVE_RAMP_INPUT *l_SCurve_input);

INT16 aDynamicScurve(FLOAT Target_val, T_SCURVE_RAMP *l_S_ramp);

#endif /* ADYNAMICSCURVE_H_ */
