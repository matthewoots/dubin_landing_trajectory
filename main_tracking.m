clc
clear all
close all

% Include bspline library
addpath('bspline');

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

order = 3;
knot_factor = 5; % Factor for how many divisions in one knot

knot_span = timeint * knot_factor; % Changes the dist estimation for 1 knot

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

%% Main function

[path,nCi,nCf,flag,segment] = dubinTimeBased(ip, lineup_point, ib, fb, minTurnRad, dist_int, flightHeight);
if ~flag
    return
end

wp = [lineup_point; descend_point; buffer_point; fp];

[cp_tmp,wp_t] = uniformSeperation(wp, Vconst, knot_span);
cp_tmp = [path', cp_tmp, fp'];
timespan = [wp_t(1), wp_t(end)];

cp = zeros(3,length(cp_tmp) + 2*(order-1));
for j = 1:3
    % We will clamp the bspline by adding to the first and the last cp
    cp(j,:) = getClampedCP(cp_tmp(j,:), order);
    range = [order:length(cp)]; 
    intv(j) = knot_factor;
    
    [x(j+1,:), x(j+4,:), x(1,:)] = getBSpline(order, timespan, cp(j,:), intv(j), true);
end

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
plot3(ip(1),ip(2),ip(3),'.',fp(1),fp(2),fp(3),'.','MarkerSize',12);
plot3(c1(:,1),c1(:,2),h,'--', c2(:,1),c2(:,2),h,'--');
plot3(line_i(:,1),line_i(:,2),[flightHeight;flightHeight],'--');
plot3(line_f(:,1),line_f(:,2),[flightHeight;flightHeight],'--');
plot3(buffer_point(1),buffer_point(2),buffer_point(3),'.','MarkerSize',12); 
plot3(descend_point(1),descend_point(2),descend_point(3),'.','MarkerSize',12); 
plot3(lineup_point(1),lineup_point(2),lineup_point(3),'.','MarkerSize',12);

% Path
plot3(path(:,1),path(:,2),path(:,3),'x','MarkerSize',4);
plot3(x(2,:),x(3,:),x(4,:),'x','MarkerSize',4);

plot3(cp(1,:),cp(2,:),cp(3,:),'o','MarkerSize',5);

% Display settings
grid on 
axis([avgp(1)-limit(1) avgp(1)+limit(1) ...
    avgp(2)-limit(2) avgp(2)+limit(2) 0 flightHeight+10])