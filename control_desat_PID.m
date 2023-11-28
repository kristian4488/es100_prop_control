function u_thrust = control_desat_PID(error, error_i, error_d)
    Kp = 0.5;
    Ki = 0;
    Kd = 0;
    u = Kp * error + Ki * error_i + Kd * error_d;
    if error <= 0
        u_thrust = -u *[1, 0, 0, 0, 1, 0, 0, 0];
    else
        u_thrust = u * [0, 0, 0, 1, 0, 0, 0, 1];
    end
end