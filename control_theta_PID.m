% PID controller

function u_RW = control_theta_PID(error, error_i, error_d, A_RW)
    Kp = 0.01;
    Ki = 0.0001;
    Kd = 0.2;

    u_RW = -pinv(A_RW) * (Kp * error + Ki *error_i + Kd * error_d);
end