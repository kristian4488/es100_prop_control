% Slewing PID controller
% Derivative term uses quaternion derivative 

function u_RW = control_theta_PID(error, error_i, qdot, A_RW)
    Kp = 0.1;
    Ki = 0.000;
    Kd = 5;

    u_RW = -pinv(A_RW) * (Kp * error(2:4) + Ki *error_i(2:4) - Kd * qdot(2:4));
end