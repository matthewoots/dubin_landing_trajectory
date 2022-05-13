function [target, flag] = line_tracker(P, tolerance, limit_pair)
    
    deg_to_rad = 1/180 * pi;
    rad_to_deg = 1/pi * 180;

    S = limit_pair(1,:);
    E = limit_pair(2,:);

    %% Find nearest point on the line closest to the point
    % https://math.stackexchange.com/questions/1905533/find-perpendicular-distance-from-point-to-line-in-3d
    v = (E - S) / norm(E - S);
    d = dot(v,(P - S));
    np = S + v * d;

    %% Find the next point
    next_point = np + v * tolerance;
    next_point_d = norm(next_point - S);
    end_start_d = norm(E - S);

    % Check with the maximum allowed distance [next_point_d]
    flag = (end_start_d - next_point_d) > 0;
    fprintf('[lt] flag %d\n',flag);
    
    target = next_point;
end

