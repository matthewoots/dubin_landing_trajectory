function [target, flag] = planar_circle_tracker(C, P, radius, tolerance, rot, limit_pair)    
    deg_to_rad = 1/180 * pi;
    rad_to_deg = 1/pi * 180;

    % Make sure this aligns to clockwise or anticlockwise
    % The pair consist of [start end] angle
    % This is the output from dubin flight path
    fprintf('[ct] limit_pair (%.3f %.3f)\n', limit_pair(1), limit_pair(2));

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
    cf = [0 radius 0];
    % https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
    % Find angle between 2 vectors
    dotp = dot(cf, cp); % dot product between [x1, y1] and [x2, y2]
    detp = cf(2)*cp(1) - cp(2)*cf(1); % determinant
    curr_ang_rot = atan2(detp, dotp);
    fprintf('[ct] cir_ang_change %.3f deg\n',cir_ang_change/pi * 180);
    fprintf('[ct] curr_ang_rot %.3f deg\n',curr_ang_rot/pi * 180);
    next_ang = wrapToPi(curr_ang_rot + dir * cir_ang_change);
    next_point = [C(1) + radius * sin(next_ang), ...
                C(2) + radius * cos(next_ang), P(3)];

    %% Check that [next_point] is within the limits from dubin path
    % For clockwise (cw) final - initial will yield a positive value
    % For counter-clockwise (ccw) -(final - initial) will yield a positive value
    if strcmp(rot,'cw')
        angle_length = limit_pair(2) - limit_pair(1);
        wrap_check = (limit_pair(2) - limit_pair(1)) > 0;
        if ~wrap_check
            angle_length = angle_length + 2 * pi;
        end

        angle_length_path = next_ang - limit_pair(1);
        wrap_check_path = (next_ang - limit_pair(1)) > 0;
        if ~wrap_check_path
            angle_length_path = angle_length_path + 2 * pi;
        end
    else
        angle_length = -(limit_pair(2) - limit_pair(1));
        wrap_check = -(limit_pair(2) - limit_pair(1)) > 0;
        if ~wrap_check
            angle_length = angle_length + 2 * pi;
        end

        angle_length_path = -(next_ang - limit_pair(1));
        wrap_check_path = -(next_ang - limit_pair(1)) > 0;
        if ~wrap_check_path
            angle_length_path = angle_length_path + 2 * pi;
        end
    end

    fprintf('[ct] angle_length %.3fdeg\n',angle_length * rad_to_deg);

    % Check with the next angle [next_ang]
    flag = (angle_length - angle_length_path) > 0;
    target = next_point;
    fprintf('[ct] flag %d\n',flag);
    
end

