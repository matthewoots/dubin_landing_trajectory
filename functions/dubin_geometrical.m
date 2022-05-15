function [keypoints, turn_angles, length_of_legs, nCi, nCf, flag, angle_limits] = ...
                dubin_geometrical(ip, fp, ib, fb, minTurnRad, flightHeight)

    %% Constants
    g = 9.81;
    FLT_MAX = exp(37);
    flag = true;
    
    %% Setup Circles, and make decision
    Ci = zeros(2); Cf = zeros(2);
    
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
            flag = false;
            keypoints = 0;
            turn_angles = 0;
            length_of_legs = 0;
            angle_limits = 0;
            return
    end

    %% Checks

    if (dist^2 <= 4 * minTurnRad^2)
       disp('[error] Error resulting from complex number from sqrt')
       fprintf('c1 = %f, c2 = %f\n', dist^2, 4 * minTurnRad^2);
       fprintf('dist must be more than %f, currently %f\n', sqrt(4 * minTurnRad^2),dist);
       flag = false;
       keypoints = 0;
       turn_angles = 0;
       length_of_legs = 0;
       angle_limits = 0;
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
        fprintf('Turn 1 Anticlockwise\n');
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
        fprintf('Turn 1 Clockwise\n');
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
        fprintf('Turn 2 Anticlockwise\n');
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
        fprintf('Turn 2 Clockwise\n');
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
    
    line_dist = (pt(2,:) - pt(1,:)) / norm(pt(2,:) - pt(1,:));
    pt_w_h = [pt(1,:),flightHeight ;
              pt(2,:),flightHeight];
    
    keypoints = [ip; pt_w_h(1,:); pt_w_h(2,:); fp];
    turn_angles = [ang_1turn, ang_2turn];
    length_of_legs = [arclength1, line_dist, arclength2];
    
    cf = [0 1 0];
    
    cp = ip - [nCi,0]; % center to point
    dotp = dot(cf, cp); % dot product between [x1, y1] and [x2, y2]
    detp = cf(2)*cp(1) - cp(2)*cf(1); % determinant
    al1_1 = atan2(detp, dotp);
    
    cp = pt_w_h(1,:) - [nCi,0]; % center to point
    dotp = dot(cf, cp); % dot product between [x1, y1] and [x2, y2]
    detp = cf(2)*cp(1) - cp(2)*cf(1); % determinant
    al1_2 = atan2(detp, dotp);
    
    cp = pt_w_h(2,:) - [nCf,0]; % center to point
    dotp = dot(cf, cp); % dot product between [x1, y1] and [x2, y2]
    detp = cf(2)*cp(1) - cp(2)*cf(1); % determinant
    al2_1 = atan2(detp, dotp);
    
    cp = fp - [nCf,0]; % center to point
    dotp = dot(cf, cp); % dot product between [x1, y1] and [x2, y2]
    detp = cf(2)*cp(1) - cp(2)*cf(1); % determinant
    al2_2 = atan2(detp, dotp);
    
    
    angle_limits = [al1_1, al1_2; 
                    al2_1, al2_2];
end

