%% constants 
%positions are body frame quantities
para.m = 10;
Ixx = 1;
Iyy = 100;
Izz = 1;
para.x_cm = [1, 0, 0];
para.x_nozzle = zeros(8,3);

%% derived quantities
para.I = zeros(3);
para.I(1,1) = Ixx;
para.I(2,2) = Iyy;
para.I(3,3) = Izz;

%% control outputs
% body frame thrust vector 
thrust = zeros(8,3);
thrust(:,3) = ones(8,1);

%% sim run 
x0 = zeros(21,1);
x0(1:3) = [0, 0, 0];
tspan = [0, 2];
[tout, xout] = ode45(@(t, x)xdot(t, x, para, thrust), tspan, x0);
plot_sim(tout, xout, [1,2])

% x = (theta[1:3], 
%       w_b[1:3],    
%       wdot_b[1:3],
%       x_b[1:3], 
%       xdot_b[1:3], 
%       x[1:3], 
%       xdot[1:3])
% theta = [phi, theta, psi] (z y x implicit rotations)

%% equations of motion 
function dxdt = xdot(t, x, para, thrust)
    torque_b = zeros(8,3);
    force_b = zeros(8,3);
    for i= 1:8
        torque_b(i,:) = cross(para.x_nozzle(i,:) - para.x_cm, thrust(i,:));
        force_b(i,:) = thrust(i,:);                                           %duplicated for additional force later  
    end
    torque_net_b = sum(torque_b);
    force_net_b = sum(force_b);

    Tw_b_thetadot = [1, sin(x(1)) * tan(x(2)), cos(x(1)) * tan(x(2));
        0, cos(x(1)), -sin(x(1));
        0, sin(x(1))/cos(x(2)), cos(x(1))/cos(x(2))];
    Tb_e_psi = [-cos(x(3)), -sin(x(3)), 0;
                sin(x(3)), -cos(x(3)), 0;
                0,          0,          1];
    Tb_e_theta = [-cos(x(2)), 0, -sin(x(2));
                0,          1,      0;
                sin(x(2)), 0, cos(x(2))];
    Tb_e_phi = [1,      0,          0;
                0, -cos(x(1)), -sin(x(1));
                0, sin(x(1)), -cos(x(1))];
    Tb_e = Tb_e_psi * Tb_e_theta * Tb_e_phi;

    dxdt = zeros(21,1);
    dxdt(4:6) = x(7:9);
    dxdt(1:3) = Tw_b_thetadot * x(4:6);
    dxdt(7:9) = inv(para.I) * (torque_net_b.' - cross(x(4:6), para.I*x(4:6)));
   
    dxdt(10:12) = x(13:15);
    dxdt(13:15) = (force_net_b.'/para.m) - cross(x(4:6), x(13:15));
    dxdt(16:18) = Tb_e * x(13:15);
    dxdt(19:21) = Tb_e * (force_net_b.'/para.m);
end

%% plot outputs
function plot_sim(tout, xout, graph_sel) 
    %{
    graph_sel:
    1 - (phi, theta, psi)
    2 - (x, y, z)
    %}

    for i = (1:length(graph_sel))
        if graph_sel(i) == 1
            plot_theta(tout, xout)
        end
        if graph_sel(i) == 2
            plot_x(tout, xout)
        end
    end
end

%% plotting functions 
function plot_theta(tout, xout)
figure %thetaplot 
    subplot(3,1,1)
    plot(tout, xout(:,1), 'b', 'linewidth', 2)
    title("\textbf{No control: $\vec{\theta}$}", 'interpreter', 'latex')
    xlabel('time')
    ylabel("phi")
    subplot(3,1,2)
    plot(tout, xout(:,2), 'b', 'linewidth', 2)
    xlabel('time')
    ylabel("theta") 
    subplot(3,1,3)
    plot(tout, xout(:,3), 'b', 'linewidth', 2)
    xlabel('time')
    ylabel("psi")
end

function plot_x(tout, xout)
figure %thetaplot 
    subplot(3,1,1)
    plot(tout, xout(:,16), 'b', 'linewidth', 2)
    title("\textbf{No control: $\vec{x}$}", 'interpreter', 'latex')
    xlabel('time')
    ylabel("x")
    subplot(3,1,2)
    plot(tout, xout(:,17), 'b', 'linewidth', 2)
    xlabel('time')
    ylabel("y") 
    subplot(3,1,3)
    plot(tout, xout(:,18), 'b', 'linewidth', 2)
    xlabel('time')
    ylabel("z")
end