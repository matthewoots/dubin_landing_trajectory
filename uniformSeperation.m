function [cp, wp_t] = uniformSeperation(wp, des_vel, knot_span)   

    % Get seperation between waypoints
    for q = 1:height(wp)-1
        % Get 3d vector difference
        vec_diff = wp(q+1,:) - wp(q,:);
        % Get 3d vector distance 
        dist_vec_diff = sqrt(vec_diff(1)^2 + vec_diff(2)^2 + vec_diff(3)^2);
        % Upload it into segments to be used
        wp_sep(q) = dist_vec_diff;
    end

    seg_total = 0;
    % Estimated velocity is total time needed to complete
    % all the wp seperation over total time given
    est_vel = des_vel;

    % Estimation of distance to velocity
    dist = est_vel * knot_span; 
    
    % Check to see which axis has the biggest segment count
    wp_t = [];
    for q = 1:height(wp)-1
        % ceil helps to push values above 0 to 1 or more
        % or else seg count is 0 and causes an error
        seg(q) = ceil(wp_sep(q)/dist);
        % total number of segments will be according to the distance
        seg_total = seg_total + seg(q);
        wp_t(q) = seg_total * knot_span;
    end
    
    wp_t = [0 wp_t];
    
    % Number of axis
    for j = 1:3
        % Get the total count for the segments for that axis
        t_seg = sum(seg(:));
        cp_t = [];
        for q = 1:height(wp)-1
            % Why x axis would determine the rest of the axis segment
            % length?
            % It is to unsure the unification of the data, decreasing the x
            % axis length to make sure that the y and z axis have a higher
            % number of cp
            % cp sount is always +1 of segment count
            split = seg(q) + 1;
            cp0 = linspace(wp(q,j),wp(q+1,j),split);
            cp_t = [cp_t cp0(1:end-1)];
        end
        cp(j,:) = cp_t;
    end
end

