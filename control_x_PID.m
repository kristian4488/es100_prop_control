% Maneuvering, x PID controller

function u_thrust = control_x_PID(error, error_i, error_d, q)
    % setup extrinsic calls
    %=====================================
    coder.extrinsic("quatrotate");
    error_b = zeros(3,1);
    error_i_b = zeros(3,1);
    error_d_b = zeros(3,1);
    %=====================================
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
    error_b = quatrotate(q', error');
    error_i_b = quatrotate(q', error_i');
    error_d_b = quatrotate(q', error_d');

    u_thrust = (Kp * error_b(3) + Ki *error_i_b(3) + Kd * error_d_b(3)) * u_thrust;

end