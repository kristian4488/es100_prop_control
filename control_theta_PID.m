% Slewing PID controller
% Derivative term uses quaternion derivative 

function torque_t = control_theta_PID(error, error_i, rates, A_RW)

    Kp = 0.02;
    Ki = 0.000;
    Kd = 1;

    torque_t = -pinv(A_RW) * (Kp * error + Ki *error_i - Kd * rates);

end