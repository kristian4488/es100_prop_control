%% constants 
%positions are body frame quantities
para.m = 10;
Ixx = 1;
Iyy = 1;
Izz = 1;
phi_RW = pi/4;
para.x_cm = [0, 0, 0];
para.x_nozzle = zeros(8,3);
para.x_nozzle(1, :) = [1, 0, 0];
para.x_nozzle(2, :) = [-1, 0, 0];

%% derived quantities
para.I = zeros(3);
para.I(1,1) = Ixx;
para.I(2,2) = Iyy;
para.I(3,3) = Izz;
r_1 = [-1; -1; sqrt(2)*tan(phi_RW)];
r_2 = [1; -1; sqrt(2)*tan(phi_RW)];
r_3 = [1; 1; sqrt(2)*tan(phi_RW)];
r_4 = [-1; 1; sqrt(2)*tan(phi_RW)];
r_1 = r_1/norm(r_1);
r_2 = r_2/norm(r_2);
r_3 = r_3/norm(r_3);
r_4 = r_4/norm(r_4);
para.A_RW = [r_1, r_2, r_3, r_4];
para_bus_info = Simulink.Bus.createObject(para);
para_bus = evalin('base', para_bus_info.busName);

%% control outputs
% body frame thrust vector 
thrust = zeros(8,3);
rhodot = [0; 0; 0];
% body frame reaction wheel speeds

%thrust(:,3) = ones(8,1);
thrust(1,:) = [0, 0, 0];
thrust(2,:) = [0, 0, 0];

%% sim run - ODE45
x0 = zeros(21,1);
x0(1:3) = [0, 0, 0];
tspan = [0, 2];
[tout, xout] = ode45(@(t, x)plant(x, para, thrust, rhodot), tspan, x0);
plot_sim(tout, xout, [1])

%% sim run - Simulink


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