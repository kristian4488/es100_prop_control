% Maneuvering, x PID controller

function F_t = control_x_PID(error, error_i, error_d, q, para)
    % setup extrinsic calls
    %=====================================
    coder.extrinsic("quatrotate");
    error_b = zeros(3,1);
    error_i_b = zeros(3,1);
    error_d_b = zeros(3,1);
    %=====================================
    Kp = 0.5;
    Ki = 0;
    Kd = 0;

    error_b = quatrotate(q.', error.');
    error_i_b = quatrotate(q.', error_i.');
    error_d_b = quatrotate(q.', error_d.');

    % calculate thrusts for zero torque around COM
    A_zero = zeros(4);
    % hard coded again here to differentiate from true offset COM
    x_cm_model = [0.11294, 0.07129, 0.13034];
    A_zero((1:3), 1) = cross(para.x_nozzle(2,:) - x_cm_model, para.A_thrust(2,:));
    A_zero((1:3), 2) = cross(para.x_nozzle(3,:) - x_cm_model, para.A_thrust(3,:));
    A_zero((1:3), 3) = cross(para.x_nozzle(6,:) - x_cm_model, para.A_thrust(6,:));
    A_zero((1:3), 4) = cross(para.x_nozzle(7,:) - x_cm_model, para.A_thrust(7,:));
    A_zero(3, :) = [1, 0, 0, -1];
    A_zero(4, :) = [1, 0, 0, 0];
    B_zero = [0;0;0;1];
    F_t_zero = linsolve(A_zero, B_zero);
    F_t_zero_scale = 0.75 * para.thrust_max / max(F_t_zero);
    F_t_zero = F_t_zero * F_t_zero_scale;

    z_PID = (Kp * error_b(3) + Ki *error_i_b(3) + Kd * error_d_b(3));
    x_correct =  (Kp * error_b(1) + Ki *error_i_b(1) + Kd * error_d_b(1));
    y_correct =  (Kp * error_b(2) + Ki *error_i_b(2) + Kd * error_d_b(2));
    F_t = z_PID * [0, F_t_zero(1), F_t_zero(2), 0, 0, F_t_zero(3), F_t_zero(4), 0]; %apply to Z-axial thrusters
    F_t = F_t + x_correct * 0.01 * [0, -1, -1, 0, 0, 1, 1, 0];
    F_t = F_t + y_correct * 0.01 *[0, -1, 1, 0, 0, 1, -1, 0];

end