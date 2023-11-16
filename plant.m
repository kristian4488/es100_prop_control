% Rigid body system plant with the state vector:
% x = (q[1:4],        
%       w_b[5:7],    
%       rho[8:10],
%       x_b[11:13], 
%       xdot_b[14:16], 
%       x[17:19], 
%       xdot[20:22])

function dxdt = plant(x, para, thrust, rhodot, torque_ext_b)
    % extrinsic declarations for simulink code generation
    %====================================================
    coder.extrinsic("quatinv");
    coder.extrinsic("quatrotate");
    %====================================================
    torque_b = zeros(8,3);
    force_b = zeros(8,3);
    for i= 1:8
        torque_b(i,:) = cross((para.x_nozzle(i,:) - para.x_cm), thrust(i,:));
        force_b(i,:) = thrust(i,:);                                           %duplicated for additional force later  
    end
    torque_net_b = sum(torque_b);
    torque_net_b = torque_net_b.' + torque_ext_b;
    force_net_b = sum(force_b);
    force_net_b = force_net_b.';
    Tw_b_quat = [0,         -x(5),  -x(6),      -x(7);
                 x(5),      0,      x(7),       -x(6);
                 x(6),      -x(7),  0,          x(5);
                 x(7),      x(6),   -x(5),      0];
    dxdt = zeros(22,1);
    dxdt(1:4) = 0.5 * Tw_b_quat * x(1:4);
    dxdt(5:7) = inv(para.I) * (torque_net_b - rhodot - cross(x(5:7), para.I*x(5:7)+x(8:10)));
    dxdt(8:10) = rhodot;
    dxdt(11:13) = x(14:16);
    dxdt(14:16) = (force_net_b/para.m) - cross(x(5:7), x(14:16));
    dxdt(17:19) = quatrotate(quatinv(x(1:4)') , x(14:16)');
    dxdt(20:22) = quatrotate(quatinv(x(1:4)') , (force_net_b/para.m)');
end