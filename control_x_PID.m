% Maneuvering, x PID controller

function F_t = control_x_PID(error, error_i, error_d, q)
    % setup extrinsic calls
    %=====================================
    coder.extrinsic("quatrotate");
    error_b = zeros(3,1);
    error_i_b = zeros(3,1);
    error_d_b = zeros(3,1);
    %=====================================
    Kp = 1;
    Ki = 0.000;
    Kd = 30;

    error_b = quatrotate(q', error');
    error_i_b = quatrotate(q', error_i');
    error_d_b = quatrotate(q', error_d');

    % PID z direction output thrust assuming balanced COM
    z_balance = (Kp * error_b(3) + Ki *error_i_b(3) + Kd * error_d_b(3));
    F_t = z_balance * [0, 1, 1, 0, 0, 1, 1, 0]; %apply to Z-axial thrusters

end