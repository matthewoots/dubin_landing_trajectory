function next_idx = next_idx_tracker_circle(idx, C, P, radius, tolerance, wp)
    % Find distance of CP
    % https://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point
    cp = C - P;
    mag_cp = sqrt(cp(1)^2 + cp(2)^2);
    nearest_point = C + cp / mag_cp * radius;
    p_np = P - nearest_point;
    
    if mag_cp < radius
        % Inside Circle
        % ctp will be a
        ctp_first = true;
        
    elseif mag >= radius
        % Outside Circle
        % ctp will be b
        ctp_first = false;
        
    end
    
    % Clockwise direction is needed
    % Inside = p_ctp -> p_np
    % Outside = p_np -> p_ctp 
    % Use clockwise dot product
    
    angle = 0;
    next_idx = idx - 1;
    while angle < tolerance
        next_idx = next_idx + 1;
        curr_track_p = wp(idx, :);
        
        p_ctp = P - curr_track_p;
        dot_num = p_np(1) * p_ctp(1) + p_np(2) * p_ctp(2);
        dot_den = norm(p_np) * norm(p_ctp);    

        angle = acos(dot_num / dot_den);
        
        % a x b since we need to know in comparison 
        % which is clockwise and anticlockwise
        if ctp_first
            sign_value =  p_ctp(1) * p_np(2) - p_ctp(2) * p_np(1);
        elseif ~ctp_first
            sign_value =  p_np(1) * p_ctp(2) - p_np(2) * p_ctp(1);
        end
        
        if sign_value == 0
            % There is no rotation
            angle = angle;
        else
            sign = sign_value / abs(sign_value);
            angle = sign * angle;
        end
            
         
    end
    
end

