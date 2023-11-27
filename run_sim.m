%% constants 
% CaliPER physical info================================================
%positions are bus frame quantities
para.m = 9.7;
para.I = zeros(3);
para.I(2,2) = 0.157547;
para.I(2,3) = 0.02643496;
para.I(3,2) = 0.02643496;
para.I(3,3) = 0.11402;
para.I(3,1) = -0.0000427;
para.I(1,3) = -0.0000427;
para.I(1,1) = 0.106747;
para.I(1,2) = 0.0000713;
para.I(2,1) = 0.0000713;
%RW cant angles
theta_i = deg2rad(43);
theta_j = deg2rad(25);
theta_k = deg2rad(48);
%RW Moment of Inertia
para.J_RW = 1.8*10^-4;
%Center of Mass
para.x_cm = [0.11294, 0.07129, 0.13034];
%Center of Pressure
para.x_cp = [0.1, 0.05, 0.15];
%Nozzle Locations
para.x_nozzle = [0.20215, 0.1, 0.00903;
                 0.2138, 0.075, 0;
                 0.2138, 0.025, 0;
                 0.20215, 0, 0.00903;
                 0.02415, 0, 0.00903;
                 0.0125, 0.025, 0;
                 0.0125, 0.075, 0;
                 0.02415, 0.1, 0.00903];
%Thrust Directions
para.A_thrust = [0,-1,0;
                0, 0, 1;
                0, 0, 1;
                0, 1, 0;
                0, 1, 0;
                0, 0, 1;
                0, 0, 1;
                0, -1, 0];
%Thrust at Max Duty Cycle
para.thrust_max = 0.136;
%Worst case cross Sectional Area
para.A_s = 0.2 * 0.3;

% environmental const==================================================
para.q = 0.6;

%% derived quantities
A_RWx = cos(theta_i)/cos(theta_k);
A_RWy = 1;
A_RWz = cos(theta_j)/cos(theta_k);
r_1_bus = [-A_RWx; A_RWy; -A_RWz];
r_2_bus = [-A_RWx; A_RWy; A_RWz];
r_3_bus = [A_RWx; A_RWy; A_RWz];
r_4_bus = [A_RWx; A_RWy; -A_RWz];
r_1_bus = r_1_bus/norm(r_1_bus);
r_2_bus = r_2_bus/norm(r_2_bus);
r_3_bus = r_3_bus/norm(r_3_bus);
r_4_bus = r_4_bus/norm(r_4_bus);
para.A_RW = [r_1_bus, r_2_bus, r_3_bus, r_4_bus];

% create bussesS
para_bus_info = Simulink.Bus.createObject(para);
para_bus = evalin('base', para_bus_info.busName);

%% setup sim
% body frame thrust vector 
thrust = zeros(8,3);
rhodot = [0; 0; 0];
x0 = zeros(22,1);
% initial orientation, ZYX order, input [psi, theta, phi]
eul0 = [0, 0, 0];   
torque_ext_b = [0;0;0];
x0(1:4) = eul2quat(eul0);
% control loop reference values
% random throughout valid euler angles for testing
theta_ref = rand(3,1);
theta_ref(1) = theta_ref(1) * 2 * pi - pi;
theta_ref(2) = theta_ref(2) * pi - pi / 2;
theta_ref(3) = theta_ref(3) * 2 * pi - pi;
theta_ref = [2.9210;-1.0757;2.9568];
v_ref = [0;1;1];
% control loop selection
% 1 - slew
% 2 - maneuver v
mode = 1;
start_time = 0;
end_time = 200;
% select data to plot 
% options listed in plot_sim()
plot_sel = 'a';
% output data log path
out_path = '../runs/quat/';

%% sim run - ODE45
%{
thrust(1,:) = 1 * [0, -1, 0];
thrust(2,:) = 0 * [0, 0, 1];
thrust(3,:) = 0 * [0, 0, 1];
thrust(4,:) = 0 * [0, 1, 0];
thrust(5,:) = 1 * [0, 1, 0];
thrust(6,:) = 0 * [0, 0, 1];
thrust(7,:) = 0 * [0, 0, 1];
thrust(8,:) = 0 * [0, -1, 0];
tspan = [start_time, end_time];
[tout, xout] = ode45(@(t, x)plant(x, para, thrust, rhodot, torque_ext_b), tspan, x0);
tout = tout.';
xout = xout.';
plot_sim(tout, xout, plot_sel)
%}

%% sim run - Simulink
paramStruct.StartTime = num2str(start_time);
paramStruct.StopTime = num2str(end_time);
tic
out = sim('controller.slx', paramStruct);
run_time = toc;

%retrieve data 
x_sim = out.xout_sim.';
t_sim = out.tout_sim.';

% write outputs
switch mode
    case 1
        filetag = "slew_";
        for i = 1:length(theta_ref)
            filetag = filetag + theta_ref(i) + "_";
        end
    case 2
        filetag = "man_v_";
        for i = 1:length(theta_ref)
            filetag = filetag + x_ref(i) + "_";
        end
end
filetag = filetag + num2str(end_time) + "_";
fileID = fopen(strcat(out_path, filetag, 'timer.txt'),'w');
fprintf(fileID,'%f\n',run_time);
fclose(fileID);
data_file = strcat(out_path, filetag, 'data.mat');
save(data_file, 'x0', 'x_sim');
plot_sim(t_sim, x_sim, plot_sel)

%% plot outputs
function plot_sim(tout, xout, graph_sel) 
    %{
    graph_sel:
    n - none
    t - (phi, theta, psi)
    x - (x, y, z)
    a - all
    %}
    switch graph_sel
        case 'n'
        case 't'
            plot_theta(tout, xout)
        case 'x'
            plot_x(tout, xout)
        case 'a'
            plot_theta(tout, xout)
            plot_x(tout, xout)
        otherwise
            disp('Unrecognized plot_sel')
    end
end

%% plotting functions 
function plot_theta(tout, xout)
figure 
    eul = quat2eul(xout(1:4 , :).');
    subplot(3,1,1)
    plot(tout, eul(:,3), 'b', 'linewidth', 2)
    title("\textbf{$\vec{\theta}$}", 'interpreter', 'latex')
    xlabel('time')
    ylabel("phi")
    subplot(3,1,2)
    plot(tout, eul(:,2), 'b', 'linewidth', 2)
    xlabel('time')
    ylabel("theta") 
    subplot(3,1,3)
    plot(tout, eul(:,1), 'b', 'linewidth', 2)
    xlabel('time')
    ylabel("psi")
end

function plot_x(tout, xout)
figure 
    subplot(3,1,1)
    plot(tout, xout(17,:), 'b', 'linewidth', 2)
    title("\textbf{$\vec{x}$}", 'interpreter', 'latex')
    xlabel('time')
    ylabel("x")
    subplot(3,1,2)
    plot(tout, xout(18,:), 'b', 'linewidth', 2)
    xlabel('time')
    ylabel("y") 
    subplot(3,1,3)
    plot(tout, xout(19,:), 'b', 'linewidth', 2)
    xlabel('time')
    ylabel("z")
end