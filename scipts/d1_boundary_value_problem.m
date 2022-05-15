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
Vconst = 15; % this changes the overshoot of the curve
total_time = 4; % this changes the overshoot of the curve
add_vel_vect = [0,0,-1];

ib_deg = -20; fb_deg = 0;
timeint = 0.2;
range = 10;
flightHeight = 20;
flare_height = 10;
descend_angle = 30;
descend_angle_rad = descend_angle/180 * pi;

rollmax = 25 / 180 * pi;
lineup_dist = 40;
buffer_dist = 15;

bnd = 45;

% Get [Final] Positions
fp = [12.5,12.5,0];

% Get [Initial] and [Final] Bearing
ib = ib_deg/180 * pi; fb = fb_deg/180 * pi;


%% Construct landing parameters and intermediate point
% Point of buffer, behind final point
buffer_point = [fp(1) - buffer_dist * sin(fb), ...
    fp(2) - buffer_dist * cos(fb), 0];

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
limit_pair_acc = [0, 0, 0; 0, 0, 0];

%% Main function
dt = timeint;
runtime = total_time;
T = runtime;

p0 = limit_pair(1,:)';
v0 = limit_pair_vel(1,:)';
a0 = limit_pair_acc(1,:)';

pf = limit_pair(2,:)';
vf = limit_pair_vel(2,:)';
af = limit_pair_acc(2,:)';

timeint = linspace(0,runtime,runtime/dt);

% s0 = [p0, v0, a0];
% sf = [pf, vf, af];

delta =  [(pf - p0 -v0 * T - 0.5 * a0 * T^2) ; ...
          (vf - v0 -a0 * T) ; ...
          (af - a0)];
m = [720, -360*T, 60*T^2 ; -360*T, 168*T^2, -24*T^3 ; 60*T^2, -24*T^3, 3*T^4];
M = zeros(length(m)*width(m),length(m)*width(m));
for i = 1:length(m)*width(m)
    M1 = eye(3);
    l = mod(i,length(m)) + length(m)*(~mod(i,length(m)));
    h = ceil(i/3);
    M1(M1==1) = m(l,h);
    %  fprintf('(%d) %d, %d\n',i,1+(l-1)*length(m),1+(h-1)*length(m));
    M(1+(l-1)*length(m):1+(l-1)*length(m)+2,1+(h-1)*length(m):1+(h-1)*length(m)+2) = M1;
end

abg = 1/T^5 * M * delta;
alpha = abg(1:3); beta = abg(4:6); gamma = abg(7:9);

for ts = 1:width(timeint)
    t = timeint(ts);
    sOpt(:,ts) = [(alpha/120 * t^5 + beta/24 * t^4 + gamma/6 * t^3 + a0/2 * t^2 + v0 * t + p0); ...
                     (alpha/24 * t^4 + beta/6 * t^3 + gamma/2 * t^2 + a0 * t + v0); ...
                     (alpha/6 * t^3 + beta/2 * t^2 + gamma * t + a0)];
end
    
%% Plotting
figure
% Markers
hold on
line_pair_descend = [descend_point ; buffer_point];
plot3(descend_point(1),descend_point(2),descend_point(3), ...
    'o','DisplayName','descend point');
plot3(buffer_point(1),buffer_point(2),buffer_point(3), ...
    'o','DisplayName','buffer point');
plot3(line_pair_descend(:,1),line_pair_descend(:,2),line_pair_descend(:,3), ...
    '--','DisplayName','line');


plot3(limit_pair(1,1),limit_pair(1,2),limit_pair(1,3), ...
    '>','MarkerSize',5,'DisplayName','mid point');
plot3(limit_pair(:,1),limit_pair(:,2),limit_pair(:,3), ...
    '--','DisplayName','line');

plot3(vel_vector(:,1),vel_vector(:,2),vel_vector(:,3), ...
    '-','DisplayName','velocity');
plot3(sOpt(1,:), sOpt(2,:), sOpt(3,:), ...
    'x','DisplayName','optimal state');

% Display settings
grid on 
legend
xlabel('long/m')
ylabel('lat/m')
xlim([mid_point(1)-bnd mid_point(1)+bnd])
ylim([mid_point(2)-bnd mid_point(2)+bnd])
zlim([0 flightHeight])
view(3)
