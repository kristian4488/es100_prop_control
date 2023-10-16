%% constants 
%positions are body frame quantities
para.m = 10;
Ixx = 1;
Iyy = 1;
Izz = 1;
para.x_cm = [0, 0, 0];
para.x_nozzle = zeros(8,3);
para.x_nozzle(1, :) = [1, 0, 0];
para.x_nozzle(2, :) = [-1, 0, 0];

%% derived quantities
para.I = zeros(3);
para.I(1,1) = Ixx;
para.I(2,2) = Iyy;
para.I(3,3) = Izz;
para_bus_info = Simulink.Bus.createObject(para);
para_bus = evalin('base', para_bus_info.busName);

%% control outputs
% body frame thrust vector 
thrust = zeros(8,3);
% body frame reaction wheel speeds

%thrust(:,3) = ones(8,1);
thrust(1,:) = [0, 0, 1];
thrust(2,:) = [0, 0, -1];

%% sim run - ODE45
x0 = zeros(21,1);
x0(1:3) = [0, 0, pi/2];
tspan = [0, 2];
[tout, xout] = ode45(@(t, x)plant(x, para, thrust), tspan, x0);
plot_sim(tout, xout, [1])

%% sim run - Simulink

%{
dxdt_bus_info = Simulink.Bus.createObject(dxdt);
dxdt_bus = evalin('base', dxdt_bus_info.busName);
x_bus_info = Simulink.Bus.createObject(x);
x_bus = evalin('base', x_bus_info.busName);
%}

% x = (theta[1:3], 
%       w_b[1:3],    
%       wdot_b[1:3],
%       x_b[1:3], 
%       xdot_b[1:3], 
%       x[1:3], 
%       xdot[1:3])
% theta = [phi, theta, psi] (z y x implicit rotations)

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
    plot(tout, xout(:,8), 'b', 'linewidth', 2)
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