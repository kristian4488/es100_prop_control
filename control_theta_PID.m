% Slewing PID controller
% Derivative term uses quaternion derivative 

function torque_t = control_theta_PID(error, error_i, rates, A_RW)

    Kp = 0.2;
    Ki = 0.000;
    Kd = 0.4;

    torque_t = -pinv(A_RW) * (Kp * error(2:4) + Ki *error_i(2:4) - Kd * rates);
    %torque_t = (Kp * error(2:4) + Ki *error_i(2:4) - Kd * qdot(2:4));
end