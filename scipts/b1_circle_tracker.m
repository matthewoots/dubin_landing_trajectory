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
nCi = [0 0 flightHeight];

% Get position of point
for i=1:2
    p(1,i) = (rand(1) - 0.5) * minTurnRad * 2;
end
p(1,3) = flightHeight;

%% Main function

deg_to_rad = 1/180 * pi;

C = nCi;
P = p;
radius = minTurnRad;
% Clockwise means that 
rot = 'cw'; % ccw or cw
tolerance = 5; % Distance from one point to next tracking point
% Make sure this aligns to clockwise or anticlockwise
% The pair consist of [start end] angle
% This is the output from dubin flight path
limit_pair = [-90 * deg_to_rad, -120 * deg_to_rad];
start_dubin = [C(1) + radius * sin(limit_pair(1)), ...
            C(2) + radius * cos(limit_pair(1)), P(3)];
end_dubin = [C(1) + radius * sin(limit_pair(2)), ...
            C(2) + radius * cos(limit_pair(2)), P(3)];
fprintf('limit_pair (%f %f)\n', limit_pair(1), limit_pair(2));
    
%% Find nearest point on the circle perimeter closest to the point
% https://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point
cp = P - C; % center to point
cp_norm_2 = norm(cp);
np = C + (cp / cp_norm_2) * radius; % Nearest point on the circle

%% Find the next point
% Counter-clockwise is positive, clockwise is negative
dir = (strcmp(rot,'cw') - 0.5) / 0.5; % Produces -1 or 1
cir_ang_change = tolerance / radius;

% Find current point's angle
% vect1 is facing y (cf) and vect2 is facing current angle (cp)
cf = [0 minTurnRad 0];
% https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
% Find angle between 2 vectors
dot = dot(cf, cp); % dot product between [x1, y1] and [x2, y2]
det = cf(2)*cp(1) - cp(2)*cf(1); % determinant
curr_ang_rot = atan2(det, dot);
fprintf('cir_ang_change %f\n',cir_ang_change/pi * 180);
fprintf('curr_ang_rot %f\n',curr_ang_rot/pi * 180);
next_ang = wrapToPi(curr_ang_rot + dir * cir_ang_change);
next_point = [C(1) + radius * sin(next_ang), ...
            C(2) + radius * cos(next_ang), P(3)];

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
plot3(np(1,1),np(1,2),np(1,3),'x','MarkerSize',5,'DisplayName','nearest point');
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