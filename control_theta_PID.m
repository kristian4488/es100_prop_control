% Slewing PID controller
% Derivative term uses euler rates 

function u_RW = control_theta_PID(error, error_i, thetadot, A_RW)
    Kp = 0.1;
    Ki = 0.000;
    Kd = 5;

    u_RW = -pinv(A_RW) * (Kp * error + Ki *error_i - Kd * thetadot);
end