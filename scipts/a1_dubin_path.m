clc
clear all
close all

%% Setup

% Constants
g = 9.81;
FLT_MAX = exp(37);

% We are using ENU frame
% Bearing 0 means facing y direction
% Give [Initial] and [Final] Bearing
ib_deg = -20; fb_deg = 0;
Vconst = 10;
rollmax = 25 / 180 * pi;
timeint = 0.2;
range = 300;
flightHeight = 30;
padding = 50;

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

dist_int = Vconst * timeint;

% Get a line to represent direction, just for plotting
line_i(1,:) = ip(1:2); line_f(1,:) = fp(1:2);
line_length = 25;
line_i(2,:) = [ip(1) + line_length * sin(ib), ...
    ip(2) + line_length * cos(ib)];
line_f(2,:) = [fp(1) + line_length * sin(fb), ...
    fp(2) + line_length * cos(fb)];

% Represented by phi in the paper
minTurnRad = Vconst^2 / (g * tan(rollmax));

Ci = zeros(2); Cf = zeros(2);

%% Setup Circles, and make decision

% Left of Initial Point
Ci(1,:) = [ip(1) - minTurnRad * cos(ib), ...
    ip(2) + minTurnRad * sin(ib)];
% Right of Initial Point
Ci(2,:) = [ip(1) + minTurnRad * cos(ib), ...
    ip(2) - minTurnRad * sin(ib)];

% Left of Final Point
Cf(1,:) = [fp(1) - minTurnRad * cos(fb), ...
    fp(2) + minTurnRad * sin(fb)];
% Right of Final Point
Cf(2,:) = [fp(1) + minTurnRad * cos(fb), ...
    fp(2) - minTurnRad * sin(fb)];

%% Circle selection
dist = FLT_MAX; % Arbitrary number
for i=1:2
    for f=1:2
        d = Cf(f,:) - Ci(i,:);
        normd = sqrt(d(1)^2 + d(2)^2);
        fprintf('%d : %f\n',((i-1)*2)+(f-1),normd);
        if (dist > normd)
            tmp_sel = ((i-1)*2)+(f-1); % to make it similar to c++
            dist = normd;
            nCi = Ci(i,:); nCf = Cf(f,:);
        end
    end
end

switch tmp_sel
    case 0
        pathDecision = ['L','L'];
        rejectCircle = [Ci(2,:);Cf(2,:)];
    case 1
        pathDecision = ['L','R'];
        rejectCircle = [Ci(2,:);Cf(1,:)];
    case 2
        pathDecision = ['R','L'];
        rejectCircle = [Ci(1,:);Cf(2,:)];
    case 3
        pathDecision = ['R','R'];
        rejectCircle = [Ci(1,:);Cf(1,:)];
    otherwise
        disp('[error] No circle configuration found')
end

%% Checks

if (dist^2 <= 4 * minTurnRad^2)
   disp('[error] Error resulting from complex number from sqrt')
   fprintf('c1 = %f, c2 = %f\n', dist^2, 4 * minTurnRad^2);
   fprintf('dist must be more than %f, currently %f\n', sqrt(4 * minTurnRad^2),dist);
   return
end

%% Some tweaking to the values according to the circles

% Must check for wrapping over here since it is not able to identify
% rotational direction
% - Clockwise (leftwards) is negative
% - Anticlockwise (rightwards) is positive
% If value is not smaller or larger in the corresponding direction, we know
% it is a wrap

ndiff = nCf - nCi;
ang_circle = atan2(ndiff(1),ndiff(2)) / pi * 180;
ac = atan2(ndiff(1),ndiff(2)); % Angle between center of circles
fprintf('Center to Center Angle = %f\n', ac);

% We have to add exceptions for RL and LR
% d = sqrt(dist^2 - 4 * minTurnRad^2);
% gamma = atan2(2 * minTurnRad, d);
gamma = acos(minTurnRad/(0.5 * dist));
if strcmp(pathDecision,'LR')
    ac = wrapToPi(ac + gamma - pi/2);
    str_dist = dist * sin(gamma);
elseif strcmp(pathDecision,'RL')
    ac = wrapToPi(ac + (pi - pi/2 - gamma));
    str_dist = dist * sin(gamma);
elseif strcmp(pathDecision,'LL')  ||  strcmp(pathDecision,'RR') 
    str_dist = dist;
end

fprintf('Straight Segment Angle = %f\n', ac);

%% We check with [pathDecision](1)
if strcmp(pathDecision(1),'L') 
    % Anticlockwise
    % Rotation offset to center pointing to point in line
    offset1 = pi/2;
    % Final must be smaller than initial value
    if (ac > ib)
        tf1 = ac - 2 * pi;
    else 
        tf1 = ac;
    end
elseif strcmp(pathDecision(1),'R')
    % Clockwise
    % Rotation offset to center pointing to point in line
    offset1 = -pi/2;
    % Final must be larger than initial value
    if (ac < ib)
        tf1 = ac + 2 * pi;
    else 
        tf1 = ac;
    end
end

%% We check with [pathDecision](2)
if strcmp(pathDecision(2),'L')
    % Anticlockwise
    % Rotation offset to center pointing to point in line
    offset2 = pi/2;
    % Final must be smaller than initial value
    if (fb > ac)
        tf2 = fb - 2 * pi;
    else 
        tf2 = fb;
    end
elseif strcmp(pathDecision(2),'R')
    % Clockwise
    % Rotation offset to center pointing to point in line
    offset2 = -pi/2;
    % Final must be larger than initial value
    if (fb < ac)
        tf2 = fb + 2 * pi;
    else 
        tf2 = fb;
    end
end

% We get the difference in bearing 
ab1 = tf1 - ib; ang_1turn = ab1 / pi *180;
ab2 = tf2 - ac; ang_2turn = ab2 / pi *180;
fprintf('1st Turn Segment Angle = %f deg\n', ang_1turn);
fprintf('2nd Turn Segment Angle = %f deg\n', ang_2turn);

% Find the arc length
arclength1 = minTurnRad * abs(ab1);
arclength2 = minTurnRad * abs(ab2);

% What is the angle of turns during the turn portion
turn1b = wrapToPi(ib + ab1);
turn2b = wrapToPi(turn1b + ab2);


%% Find points of straight line
% Bearing for 1st turn 
a1 = wrapToPi(turn1b + offset1);
% Position of Straight Line Point of 1st turn 
pt(1,:) = [nCi(1) + minTurnRad * sin(a1), ...
    nCi(2) + minTurnRad * cos(a1)];

% Bearing for 2nd turn 
a2 = wrapToPi(turn1b + offset2);
% Position of Straight Line Point of 2nd turn 
pt(2,:) = [nCf(1) + minTurnRad * sin(a2), ...
    nCf(2) + minTurnRad * cos(a2)];


%% Make segments into path
% Find the remainder that needs to be carried forward to the next segment

%% Segment 1 = 1st Turn
int_seg1 = floor(arclength1 / dist_int); 
remainder1 = arclength1 / dist_int - int_seg1;
% We convert the fitted angle into angle + direction 
turn1 = (int_seg1 * dist_int) / minTurnRad * ab1/abs(ab1);
% Find the interval for turn 1
turn1_int = turn1 / int_seg1;
tmp = ib; size_path = 0;

% Loop to fit the calculated values into a path 1
for i=1:int_seg1 + 1
    path(size_path + i,:) = ...
        [nCi(1) + minTurnRad * sin(wrapToPi(tmp + (i-1) * turn1_int + offset1)), ...
        nCi(2) + minTurnRad * cos(wrapToPi(tmp + (i-1) * turn1_int + offset1))];
end

%% Segment 2 = Straight line segment
overlap1 = (1 - remainder1) * dist_int;
fprintf('overlap1 = %f m\n', overlap1);
int_seg2 = floor((str_dist - overlap1) / dist_int);
remainder2 = (str_dist - overlap1) / dist_int - int_seg2;
size_path = size_path + int_seg1 + 1;
tmp_pos = [pt(1,1) + overlap1 * sin(ac), ...
    pt(1,2) + overlap1 * cos(ac)];

% Loop to fit the calculated values into a path 2
for i=1:int_seg2 + 1
    path(size_path + i,:) = ...
        [tmp_pos(1) + dist_int * sin(ac) * (i-1), ...
        tmp_pos(2) + dist_int * cos(ac) * (i-1)];
end

%% Segment 3 = 3rd Turn
overlap2 = (1 - remainder2) * dist_int;
fprintf('overlap2 = %f m\n', overlap2);
int_seg3 = floor((arclength2 - overlap2) / dist_int);
remainder3 = (arclength2 - overlap2) / dist_int - int_seg3;

remaining_angle = overlap2 / minTurnRad  * ab2/abs(ab2);
% We convert the fitted angle into angle + direction 
turn2 = (int_seg3 * dist_int) / minTurnRad * ab2/abs(ab2);

% Find the interval for turn 2 
turn2_int = turn2 / int_seg3;
tmp = ac + remaining_angle; 
size_path = size_path + int_seg2 + 1;

% Loop to fit the calculated values into a path 3
for i=1:int_seg3 + 1
    path(size_path + i,:) = ...
        [nCf(1) + minTurnRad * sin(wrapToPi(tmp + (i-1) * turn2_int + offset2)), ...
        nCf(2) + minTurnRad * cos(wrapToPi(tmp + (i-1) * turn2_int + offset2))];
end

h_path = flightHeight + zeros(1, length(path));

path(:,3) = h_path;

%% Draw the Circle
c = linspace(-pi,pi,20);
h = flightHeight + zeros(1, length(c));

c1(:,1) = nCi(1) + minTurnRad .* sin(c);
c1(:,2) = nCi(2) + minTurnRad .* cos(c);

c2(:,1) = nCf(1) + minTurnRad .* sin(c);
c2(:,2) = nCf(2) + minTurnRad .* cos(c);

r1(:,1) = rejectCircle(1,1) + minTurnRad .* sin(c);
r1(:,2) = rejectCircle(1,2) + minTurnRad .* cos(c);

r2(:,1) = rejectCircle(2,1) + minTurnRad .* sin(c);
r2(:,2) = rejectCircle(2,2) + minTurnRad .* cos(c);

%% Plotting
figure
% Markers
plot3(ip(1),ip(2),ip(3),'.',fp(1),fp(2),fp(3),'.','MarkerSize',12);
hold on
% Markers
plot3(nCi(1),nCi(2),flightHeight,'x',nCf(1),nCf(2),flightHeight,'x','MarkerSize',12);
plot3(pt(:,1),pt(:,2),[flightHeight;flightHeight]);
plot3(pt(1,1),pt(1,2),flightHeight,'x','MarkerSize',8);
plot3(pt(2,1),pt(2,2),flightHeight,'x','MarkerSize',8);
plot3(c1(:,1),c1(:,2),h, c2(:,1),c2(:,2),h);
plot3(line_i(:,1),line_i(:,2),[flightHeight;flightHeight],'--');
plot3(line_f(:,1),line_f(:,2),[flightHeight;flightHeight],'--');
plot3(r1(:,1),r1(:,2),h,'--',r2(:,1),r2(:,2),h,'--');
% Path
plot3(path(:,1),path(:,2),h_path,'x','MarkerSize',4)
grid on 
view(2)
axis([avgp(1)-limit(1) avgp(1)+limit(1) ...
    avgp(2)-limit(2) avgp(2)+limit(2) 0 flightHeight+10])