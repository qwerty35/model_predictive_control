close all;
clc;
clear;

% Parameter
params.Ts = 0.1;
params.V = 1;
params.u_max = 0.5;

% Initial condition
s0 = [0;3;0]; % s = [x;y;theta]
u0 = 0;

s = s0;
u_prev = u0;

% Simulation configuration
total_time = 10;
replanning_time = 0.05;
max_iter = total_time/replanning_time;
time = 0;
nstep = 10;
tstep = replanning_time/nstep;

xtraj = zeros(max_iter*nstep, length(s0));
ttraj = zeros(max_iter*nstep, 1);
utraj = zeros(max_iter, 2);
    
for iter = 1:max_iter
    tic
    timeint = time:tstep:time+replanning_time;
    params.u = linear_controller(s, u_prev, params);
    [tsave, ssave] = ode45(@(t,s) car_dynamics(t, s, params), timeint, s, params);

    s = ssave(end-1,:)';
    u_prev = params.u;
    
    utraj(iter,:) = [time params.u];
    straj((iter-1)*nstep+1:iter*nstep,:) = ssave(1:end-1,:);
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    
    time = time + replanning_time;
    toc
end

figure('Name', 'traj');
plot(straj(:,1), straj(:,2))
figure('Name', 'theta');
plot(ttraj, straj(:,3))
figure('Name', 'x');
plot(ttraj, straj(:,1))
figure('Name', 'y');
plot(ttraj, straj(:,2))
figure('Name', 'u')
plot(utraj(:,1), utraj(:,2))
