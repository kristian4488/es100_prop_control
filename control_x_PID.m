% Maneuvering, x PID controller

function u_thrust = control_x_PID(error, error_i, error_d)
    Kp = 0.1;
    Ki = 0.000;
    Kd = 3;

    u_thrust = [0,0,0;
                0, 0, 0.25;
                0, 0, 0.25;
                0,0,0;
                0,0,0;
                0, 0, 0.25;
                0, 0, 0.25;
                0,0,0];
    if error(3) > 0
        u_thrust = (Kp * error(3) + Ki *error_i(3) + Kd * error_d(3)) * u_thrust;
    else
        u_thrust = zeros(8,3);
    end
    
end