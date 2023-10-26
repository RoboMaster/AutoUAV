// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#include "pid.h"

RM::PIDController::PIDController(float _p, float _i, float _d, float _limit)
{
    p=_p;
    i=_i;
    d=_d;
    limit=_limit;
    last_error=0;
    previous_error=0;
    current_error=0;
    last_output=0;
}


float RM::PIDController::compute(float actual_u, float expect_u)
{
    current_error = expect_u - actual_u;
 
	float output =  last_output + p * (current_error - last_error)  
			  + i*current_error      
			  + d * (current_error - 2 * last_error + previous_error);  
	
	previous_error = last_error;
	last_error = current_error;	  	
	if(output > limit)output=limit;
    else if(output < -limit)output=-1*limit;
    last_output = output;
	return output;

}
