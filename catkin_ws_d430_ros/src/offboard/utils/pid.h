// #  Copyright © 2023 ROBOMASTER All Rights Reserved.
// #  You may use, distribute and modify this code under the
// #  terms of the MIT license, which unfortunately won't be
// #  written for another century.

#pragma once

namespace RM
{
class PIDController
{
    public:
    PIDController(float p, float i, float d, float limit);
    float compute(float actual_u, float expect_u);

    private:
    float p, i, d, limit;
    float current_error, last_error, previous_error;
    float last_output;
};
}
