clc
clear all
close all

% Include library
prefix = '..';
addpath(strcat(prefix, '/functions'));

%% Introduction
% 1. [Dubin] path to align the FW to the heading
% 2. [Lineup] acts as a buffer to readjust to heading 
% 3. [Descend] according to a slope
% 4. [Flare] stage will require some form of tracking to make sure it stops
% at the desired landing position

%% Setup

% Constants
g = 9.81;
FLT_MAX = exp(37);

% We are using ENU frame
% Bearing 0 means facing y direction
Vconst = 20; % this changes the overshoot of the curve
total_time = 6; % this changes the overshoot of the curve
additional_curve_height = 2; % this changes the overshoot of the curve

ib_deg = -20; fb_deg = -20;
timeint = 0.1;
range = 10;
flightHeight = 20;
flare_height = 6;
descend_angle = 30;
descend_angle_rad = descend_angle/180 * pi;

rollmax = 25 / 180 * pi;
lineup_dist = 40;
buffer_dist = 15;

bnd = 45;

landing_height = 0.3; % elevated
% Get [Final] Positions
fp = [12.5,12.5,landing_height];

% Get [Initial] and [Final] Bearing
ib = ib_deg/180 * pi; fb = fb_deg/180 * pi;


%% Construct landing parameters and intermediate point
% Point of buffer, behind final point
buffer_point = [fp(1) - buffer_dist * sin(fb), ...
    fp(2) - buffer_dist * cos(fb), landing_height + additional_curve_height];

% Point of descend, behind buffer point
descend_dist = flightHeight / sin(descend_angle_rad);
descend_point = [buffer_point(1) - descend_dist * sin(fb), ...
    buffer_point(2) - descend_dist * cos(fb), flightHeight];

% Point of lineup, behind descend point
lineup_point = [descend_point(1) - lineup_dist * sin(fb), ...
    descend_point(2) - lineup_dist * cos(fb), flightHeight];


% Represented by phi in the paper
minTurnRad = Vconst^2 / (g * tan(rollmax));

%% Start of constructing vectors and lines
% This is the output from dubin flight path
bd = buffer_point - descend_point;
bd_v = bd / norm(bd);
% Find the distance to the mid point
dist_to_mid = (flightHeight - flare_height) / sin(descend_angle_rad);
mid_point = descend_point + dist_to_mid * bd_v;

limit_pair = [mid_point; fp];
incline = pi/2 - atan(norm(bd_v(1:2))/abs(bd_v(3)));
incline_deg = incline/pi * 180;
land_vel = bd_v * Vconst;
vel_vector = [limit_pair(1,:); limit_pair(1,:) + bd_v * 10]; % for plotting the vector line

limit_pair_vel = [land_vel ; 0, 0, 0];
% limit_pair_acc = [land_vel / 3; 0, 0, 0];
limit_pair_acc = [0, 0, 0; 0, 0, 0];

%% Main function
tic
pass_checks = false;
dt = timeint;
runtime = total_time;
iter = 1;

s0(1:3,1) = limit_pair(1,:)';
s0(4:6,1) = limit_pair_vel(1,:)';
s0(7:9,1) = limit_pair_acc(1,:)';

d0(1:3,1) = limit_pair(2,:)';
d0(4:6,1) = limit_pair_vel(2,:)';
d0(7:9,1) = limit_pair_acc(2,:)';

%% Original BVP without finding landing optimal curve
[intervals_original, sOpt_original] = ...
    boundary_value_problem(timeint, runtime, d0, s0);

%% BVP with finding landing optimal curve
while (~pass_checks)
    sOpt = [];
    [intervals, sOpt] = ...
        boundary_value_problem(timeint, runtime, d0, s0);
    %% Check pass criteria
    % If z velocity is found to have a positive value we will continue here
    % and not update pass_checks
    if sum(sOpt(6,:) > 0.001) > 0
        runtime = runtime - timeint;
        iter = iter + 1;
        continue; 
    end

    pass_checks = true;
end

fprintf('time %f iter %d\n',toc, iter);
    
%% Plotting
figure(1)
% Markers
hold on
line_pair_descend = [descend_point ; buffer_point];
line_pair_buffer = [buffer_point ; fp];
plot3(descend_point(1),descend_point(2),descend_point(3), ...
    'o','DisplayName','descend point');
plot3(buffer_point(1),buffer_point(2),buffer_point(3), ...
    'o','DisplayName','buffer point');
plot3(line_pair_descend(:,1),line_pair_descend(:,2),line_pair_descend(:,3), ...
    '-','LineWidth', 2,'DisplayName','descend to buffer line');


plot3(limit_pair(1,1),limit_pair(1,2),limit_pair(1,3), ...
    '>','MarkerSize',7,'DisplayName','mid point');
plot3(limit_pair(:,1),limit_pair(:,2),limit_pair(:,3), ...
    '-','LineWidth', 2,'DisplayName','mid to final line');
plot3(line_pair_buffer(:,1),line_pair_buffer(:,2),line_pair_buffer(:,3), ...
    '-','LineWidth', 2,'DisplayName','buffer to final line');

% plot3(vel_vector(:,1),vel_vector(:,2),vel_vector(:,3), ...
%     '-','DisplayName','velocity vector');
plot3(sOpt(1,:), sOpt(2,:), sOpt(3,:), ...
    'x','DisplayName','optimal state','MarkerSize',3);
plot3(sOpt_original(1,:), sOpt_original(2,:), sOpt_original(3,:), ...
    'o','DisplayName','original guess state','MarkerSize',3);

% Display settings
hold off
grid on 
legend
xlabel('long/m')
ylabel('lat/m')
zlabel('alt/m')
xlim([mid_point(1)-bnd mid_point(1)+bnd])
ylim([mid_point(2)-bnd mid_point(2)+bnd])
zlim([0 flightHeight])
view(3)

%% Plot Boundary Value Problem results in terms of velocity and acceleration
figure(2)
subplot(3,1,1)
for i = 1:length(sOpt)
    total_velocity(i) = norm(sOpt(4:6,i));  
end
for i = 1:length(sOpt_original)
    total_velocity_original(i) = norm(sOpt_original(4:6,i));
end
hold on
plot(intervals,total_velocity,'DisplayName','optimal');
plot(intervals_original,total_velocity_original,'DisplayName','original');
hold off
xlabel('t [s]'); ylabel('Vel [m/s]');
grid on
legend
title(sprintf('Total Velocity (direction not included)'));

subplot(3,1,2)
hold on
plot(intervals,sOpt(4,:),'DisplayName','X/s');
plot(intervals,sOpt(5,:),'DisplayName','Y/s');
plot(intervals,sOpt(6,:),'DisplayName','Z/s');
hold off
xlabel('t [s]'); ylabel('Vel [m/s^2]');
grid on
legend
title(sprintf('Velocity'));

subplot(3,1,3)
for i = 1:length(sOpt)
    total_acc(i) = norm(sOpt(7:9,i));
end
for i = 1:length(sOpt_original)
    total_acc_original(i) = norm(sOpt_original(7:9,i));
end
hold on
plot(intervals,total_acc,'DisplayName','optimal');
plot(intervals_original,total_acc_original,'DisplayName','original');
hold off
xlabel('t [s]'); ylabel('acc [m/s^2]');
grid on
legend
title(sprintf('Total Acceleration (direction not included)'));
