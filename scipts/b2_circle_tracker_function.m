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
nCi = [4 4 flightHeight];

% Get position of point
for i=1:2
    p(1,i) = (rand(1) - 0.5) * minTurnRad * 2;
end
p(1,3) = flightHeight;

%% Main function

deg_to_rad = 1/180 * pi;
rad_to_deg = 1/pi * 180;

C = nCi;
P = p;
radius = minTurnRad;
rot = 'ccw'; % ccw or cw
tolerance = 5; % Distance from one point to next tracking point
% Make sure this aligns to clockwise or anticlockwise
% The pair consist of [start end] angle
% This is the output from dubin flight path
limit_pair = [-90 * deg_to_rad, 120 * deg_to_rad];
start_dubin = [C(1) + radius * sin(limit_pair(1)), ...
            C(2) + radius * cos(limit_pair(1)), P(3)];
end_dubin = [C(1) + radius * sin(limit_pair(2)), ...
            C(2) + radius * cos(limit_pair(2)), P(3)];
fprintf('[ct] limit_pair (%.3f %.3f)\n', limit_pair(1), limit_pair(2));

%% Tracker function
[next_point, flag] = planar_circle_tracker(C, P, radius, tolerance, rot, limit_pair);

%% To plot the track line from current to next point
track_line = [p; next_point];


%% Draw the Circle
c = linspace(-pi,pi,20);
h = flightHeight + zeros(1, length(c));

c1(:,1) = nCi(1) + minTurnRad .* sin(c);
c1(:,2) = nCi(2) + minTurnRad .* cos(c);

%% Plotting
figure
% Markers
hold on
plot3(c1(:,1),c1(:,2),h,'--','DisplayName','perimeter');
plot3(nCi(1),nCi(2), nCi(3),'x','DisplayName','center');
plot3(p(1,1),p(1,2),p(1,3),'o','DisplayName','random point');
plot3(start_dubin(1,1),start_dubin(1,2),start_dubin(1,3), ...
    'o','DisplayName','start point');
plot3(end_dubin(1,1),end_dubin(1,2),end_dubin(1,3), ...
    '>','MarkerSize',5,'DisplayName','end point');
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
view(2)