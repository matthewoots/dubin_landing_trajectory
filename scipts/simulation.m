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
% Give [Initial] and [Final] Bearing
ib_deg = -20; fb_deg = 0;
Vconst = 15;

timeint = 0.2;
range = 400;
flightHeight = 20;
padding = 150;

rollmax = 25 / 180 * pi;
descendAngle = 25 / 180 * pi;

lineup_dist = 40;
buffer_dist = 15;

% Get [Initial] and [Final] Positions
for i=1:2
    ip(i) = (rand(1) - 0.5) * range;
    fp(i) = (rand(1) - 0.5) * range;
end
ip(3) = flightHeight; fp(3) = 0;

avgp = (ip + fp) / 2;
limit = abs(avgp - ip) + padding;

% Get [Initial] and [Final] Bearing
ib = ib_deg/180 * pi; fb = fb_deg/180 * pi;

fprintf('Initial: [%f %f] %f rad\n', ip(1), ip(2), ib);
fprintf('Final: [%f %f] %f rad\n', fp(1), fp(2), fb);


%% Construct landing parameters and intermediate point
% Point of buffer, behind final point
buffer_point = [fp(1) - buffer_dist * sin(fb), ...
    fp(2) - buffer_dist * cos(fb), 0];

% Point of descend, behind buffer point
descend_dist = flightHeight / sin(descendAngle);
descend_point = [buffer_point(1) - descend_dist * sin(fb), ...
    buffer_point(2) - descend_dist * cos(fb), flightHeight];

% Point of lineup, behind descend point
lineup_point = [descend_point(1) - lineup_dist * sin(fb), ...
    descend_point(2) - lineup_dist * cos(fb), flightHeight];

% dist_int = Vconst * timeint;
dist_int = Vconst;

% Get a line to represent direction, just for plotting
line_i(1,:) = ip(1:2); line_f(1,:) = lineup_point(1:2);
line_length = 25;
line_i(2,:) = [ip(1) + line_length * sin(ib), ...
    ip(2) + line_length * cos(ib)];
line_f(2,:) = [lineup_point(1) + line_length * sin(fb), ...
    lineup_point(2) + line_length * cos(fb)];


% Represented by phi in the paper
minTurnRad = Vconst^2 / (g * tan(rollmax));

%% Main function to construct movement of UAV
% (1) To construct the path for the dubin path until the lineup point
[seg1_path,nCi,nCf,flag,segment] = dubin_time_based(ip, lineup_point, ib, fb, minTurnRad, dist_int, flightHeight);
if ~flag
    return
end

% (2) To construct the path after the dubin path till the end
ending_segment = [lineup_point; descend_point; buffer_point; fp];
[seg2_path, wp_t] = uniform_seperation(ending_segment, Vconst, 1);
% With uniform seperation, the last point is not set hence we have to add
% it back
seg2_path(:,end+1) = fp';
path = [seg1_path ; seg2_path'];


%% Draw the Circle
c = linspace(-pi,pi,20);
h = flightHeight + zeros(1, length(c));

c1(:,1) = nCi(1) + minTurnRad .* sin(c);
c1(:,2) = nCi(2) + minTurnRad .* cos(c);

c2(:,1) = nCf(1) + minTurnRad .* sin(c);
c2(:,2) = nCf(2) + minTurnRad .* cos(c);

%% Plotting
fig = figure;
clf % Clear figure

hold on
% Markers
plot3(ip(1),ip(2),ip(3),'x',fp(1),fp(2),fp(3), ...
    'x','MarkerSize',12,'DisplayName','start & end');
plot3(c1(:,1),c1(:,2),h,'--', c2(:,1),c2(:,2),h, ...
    '--', 'DisplayName','dubin circles');
plot3(line_i(:,1),line_i(:,2),[flightHeight;flightHeight], ...
    '--', 'DisplayName','initial heading');
plot3(line_f(:,1),line_f(:,2),[flightHeight;flightHeight], ...
    '--', 'DisplayName','final heading');
plot3(buffer_point(1),buffer_point(2),buffer_point(3), ...
    '.','MarkerSize',15, 'DisplayName','buffer point'); 
plot3(descend_point(1),descend_point(2),descend_point(3), ...
    '.','MarkerSize',15, 'DisplayName','descent point'); 
plot3(lineup_point(1),lineup_point(2),lineup_point(3), ...
    '.','MarkerSize',15, 'DisplayName','lineup heading');

% Flight path
% plot3(seg1_path(:,1),seg1_path(:,2),seg1_path(:,3),'x','MarkerSize',4);
plot3(path(:,1),path(:,2),path(:,3), ...
    'o','MarkerSize',5, 'DisplayName','path');

% Display settings
grid on 
legend
xlabel('long/m')
ylabel('lat/m')
zlabel('altitude/m');
view(3)
axis([avgp(1)-limit(1) avgp(1)+limit(1) ...
    avgp(2)-limit(2) avgp(2)+limit(2) 0 flightHeight+10]) 