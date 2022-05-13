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
Vconst = 10;

timeint = 0.2;
range = 10;
flightHeight = 20;

rollmax = 25 / 180 * pi;

% Represented by phi in the paper
minTurnRad = Vconst^2 / (g * tan(rollmax));

% This is the output from dubin flight path
limit_pair = [5 10 flightHeight; ...
            -7 -5 flightHeight/3];
bnd = 10;

%% Get position of point
range = [bnd, bnd, 2];
mid = [(limit_pair(1,1) + limit_pair(2,1))/2, ...
    (limit_pair(1,2) + limit_pair(2,2))/2, ...
    (limit_pair(1,3) + limit_pair(2,3))/2];
for i=1:3
    p(1,i) = (rand(1) - 0.5) * range(i) + mid(i);
end

%% Main function

P = p;
tolerance = 2; % Distance from one point to next tracking point
[next_point, flag] = line_tracker(P, tolerance, limit_pair);

% To plot the track line from current to next point
track_line = [p; next_point];
    
%% Plotting
figure
% Markers
hold on
plot3(p(1,1),p(1,2),p(1,3),'o','DisplayName','random point');
plot3(limit_pair(2,1),limit_pair(2,2),limit_pair(2,3), ...
    'o','DisplayName','end point');
plot3(limit_pair(1,1),limit_pair(1,2),limit_pair(1,3), ...
    '>','MarkerSize',5,'DisplayName','start point');
plot3(limit_pair(:,1),limit_pair(:,2),limit_pair(:,3), ...
    '--','DisplayName','line');
plot3(next_point(1,1), ...
    next_point(1,2), ...
    next_point(1,3), ...
    'x','MarkerSize',7,'DisplayName','next point');
plot3(track_line(:,1), ...
    track_line(:,2), ...
    track_line(:,3), ...
    '->','DisplayName','track_line');

% Display settings
grid on 
legend
xlabel('long/m')
ylabel('lat/m')
xlim([-bnd bnd])
ylim([-bnd bnd])
zlim([0 flightHeight])
view(3)