/*
 * aScurve.c
 *
 *  Created on: 15-May-2021
 *      Author: dd.avinash
 */
#include "aDynamicScurve.h"

//VOID aDynamicScurve_init(T_SCURVE_RAMP *l_S_ramp)
//{
//	l_S_ramp->SCurve_input.Ramp_up_sec = 0;
//	l_S_ramp->SCurve_input.Ramp_dn_sec = 0;
//	l_S_ramp->SCurve_input.Acc_start = 0;
//	l_S_ramp->SCurve_input.Acc_end = 0;
//	l_S_ramp->SCurve_input.Dec_start = 0;
//	l_S_ramp->SCurve_input.Dec_end = 0;
//	l_S_ramp->SCurve_input.sampling_time = 0;
//}

VOID aDynamicScurve_init(T_SCURVE_RAMP *l_S_ramp,T_SCURVE_RAMP_INPUT *l_SCurve_input)
{
	l_S_ramp->SCurve_input = *l_SCurve_input;
}

INT16 aDynamicScurve(FLOAT Target_val, T_SCURVE_RAMP *l_S_ramp)
{
	if(l_S_ramp->prev_target_val != Target_val)
	{
		l_S_ramp->acc_astr = l_S_ramp->SCurve_input.Acc_start/1000.0;
		l_S_ramp->acc_aend = l_S_ramp->SCurve_input.Acc_end/1000.0;
		l_S_ramp->dec_astr = l_S_ramp->SCurve_input.Dec_start/1000.0;
		l_S_ramp->dec_aend = l_S_ramp->SCurve_input.Dec_end/1000.0;

		l_S_ramp->total_acc_time = (Target_val - l_S_ramp->output_val)/l_S_ramp->SCurve_input.Ramp_up_sec;
		l_S_ramp->total_dec_time = (Target_val - l_S_ramp->output_val)/l_S_ramp->SCurve_input.Ramp_dn_sec;

		if(l_S_ramp->total_acc_time<0)
		{
			l_S_ramp->total_acc_time = -l_S_ramp->total_acc_time;
		}
		if(l_S_ramp->total_dec_time<0)
		{
			l_S_ramp->total_dec_time = -l_S_ramp->total_dec_time;
		}
		l_S_ramp->total_accn_time = l_S_ramp->total_acc_time - ((l_S_ramp->acc_astr + l_S_ramp->acc_aend));
		l_S_ramp->total_decn_time = l_S_ramp->total_dec_time - ((l_S_ramp->dec_astr + l_S_ramp->dec_aend));
		if(l_S_ramp->total_accn_time<0)
		{
			l_S_ramp->acc_astr = l_S_ramp->acc_aend = l_S_ramp->total_acc_time/2;
			l_S_ramp->total_accn_time = 0;
		}
		if(l_S_ramp->total_decn_time<0)
		{
			l_S_ramp->dec_astr = l_S_ramp->dec_aend = l_S_ramp->total_dec_time/2;
			l_S_ramp->total_decn_time = 0;
		}
		l_S_ramp->act_ramp_up_sec = (Target_val - l_S_ramp->output_val)/( l_S_ramp->total_accn_time + ((l_S_ramp->acc_astr + l_S_ramp->acc_aend)/2));
		l_S_ramp->act_ramp_dn_sec = (Target_val - l_S_ramp->output_val)/( l_S_ramp->total_decn_time + ((l_S_ramp->dec_astr + l_S_ramp->dec_aend)/2));
		if(l_S_ramp->act_ramp_up_sec<0)
		{
			l_S_ramp->act_ramp_up_sec = -l_S_ramp->act_ramp_up_sec;
		}
		if(l_S_ramp->act_ramp_dn_sec<0)
		{
			l_S_ramp->act_ramp_dn_sec = -l_S_ramp->act_ramp_dn_sec;
		}
		l_S_ramp->counter = 0;
		l_S_ramp->accleration_ramp_profile = 0;
	}
	// ENTER YOUR CODE HERE...
	if(Target_val>=l_S_ramp->output_val)
	{
		if(l_S_ramp->counter<(UINT32)(l_S_ramp->acc_astr/l_S_ramp->SCurve_input.sampling_time))
		{
			l_S_ramp->accleration_ramp_profile += (l_S_ramp->act_ramp_up_sec/(l_S_ramp->acc_astr))*((l_S_ramp->SCurve_input.sampling_time));

			if(l_S_ramp->accleration_ramp_profile>l_S_ramp->act_ramp_up_sec)
			{
				l_S_ramp->accleration_ramp_profile = l_S_ramp->act_ramp_up_sec;
			}

			l_S_ramp->output_val += (l_S_ramp->accleration_ramp_profile*(l_S_ramp->SCurve_input.sampling_time));
		}
		else if((l_S_ramp->counter>=(UINT32)((1/l_S_ramp->SCurve_input.sampling_time)*l_S_ramp->acc_astr))&&((l_S_ramp->counter<(UINT32)((1/l_S_ramp->SCurve_input.sampling_time)*(l_S_ramp->acc_astr + l_S_ramp->total_accn_time)))))
		{
			l_S_ramp->accleration_ramp_profile = l_S_ramp->act_ramp_up_sec;
			l_S_ramp->output_val += (l_S_ramp->accleration_ramp_profile/(1/l_S_ramp->SCurve_input.sampling_time));
		}
		else if((l_S_ramp->counter>=(UINT32)((1/l_S_ramp->SCurve_input.sampling_time)*(l_S_ramp->acc_astr + l_S_ramp->total_accn_time)))&&(l_S_ramp->counter<=(UINT32)((1/l_S_ramp->SCurve_input.sampling_time)*l_S_ramp->total_acc_time)))
		{
			l_S_ramp->accleration_ramp_profile -= (l_S_ramp->act_ramp_up_sec/(l_S_ramp->acc_aend))/((1/l_S_ramp->SCurve_input.sampling_time));

			if(l_S_ramp->accleration_ramp_profile<0)
			{
				l_S_ramp->accleration_ramp_profile = 0;
			}

			l_S_ramp->output_val += (l_S_ramp->accleration_ramp_profile/(1/l_S_ramp->SCurve_input.sampling_time));

			if(l_S_ramp->output_val>=Target_val)
			{
				l_S_ramp->output_val =Target_val;
				l_S_ramp->counter = 0;
			}
		}
		else
		{
			l_S_ramp->output_val = Target_val;
		}
	}
	if(Target_val<l_S_ramp->output_val)
	{
			if(l_S_ramp->counter<(UINT32)(1/l_S_ramp->SCurve_input.sampling_time)*l_S_ramp->dec_astr)
			{
				l_S_ramp->accleration_ramp_profile -= (l_S_ramp->act_ramp_dn_sec/(l_S_ramp->dec_astr))/((1/l_S_ramp->SCurve_input.sampling_time));
					//	out[2] = 3;
			if(l_S_ramp->accleration_ramp_profile<-l_S_ramp->act_ramp_dn_sec)
			{
				l_S_ramp->accleration_ramp_profile = -l_S_ramp->act_ramp_dn_sec;
			}

			l_S_ramp->output_val += (l_S_ramp->accleration_ramp_profile/(1/l_S_ramp->SCurve_input.sampling_time));
			}
			else if((l_S_ramp->counter>=(UINT32)((1/l_S_ramp->SCurve_input.sampling_time)*l_S_ramp->dec_astr))&&((l_S_ramp->counter<(UINT32)((1/l_S_ramp->SCurve_input.sampling_time)*(l_S_ramp->dec_astr + l_S_ramp->total_decn_time)))))
			{
				l_S_ramp->accleration_ramp_profile = -l_S_ramp->act_ramp_dn_sec;
				l_S_ramp->output_val += (l_S_ramp->accleration_ramp_profile/(1/l_S_ramp->SCurve_input.sampling_time));
			}
			else if((l_S_ramp->counter>=(UINT32)((1/l_S_ramp->SCurve_input.sampling_time)*(l_S_ramp->dec_astr + l_S_ramp->total_decn_time)))&&(l_S_ramp->counter<=(UINT32)((1/l_S_ramp->SCurve_input.sampling_time)*l_S_ramp->total_dec_time)))
			{
				l_S_ramp->accleration_ramp_profile += (l_S_ramp->act_ramp_dn_sec/(l_S_ramp->dec_aend))/((1/l_S_ramp->SCurve_input.sampling_time));

				if(l_S_ramp->accleration_ramp_profile>0)
				{
					l_S_ramp->accleration_ramp_profile = 0;
				}

				l_S_ramp->output_val += (l_S_ramp->accleration_ramp_profile/(1/l_S_ramp->SCurve_input.sampling_time));
				if((INT16)l_S_ramp->output_val<=(INT16)Target_val)
				{
					l_S_ramp->output_val =Target_val;
					l_S_ramp->counter = 0;
				}
			}
			else
			{
				l_S_ramp->output_val = Target_val;
			}
	}
	l_S_ramp->prev_target_val = Target_val ;
	if((INT16)l_S_ramp->output_val==(INT16)Target_val)
	{
		l_S_ramp->counter = 0;
		l_S_ramp->accleration_ramp_profile = 0;
	}
	else
	{
		l_S_ramp->counter++;
	}
	return (INT16)l_S_ramp->output_val;
}

