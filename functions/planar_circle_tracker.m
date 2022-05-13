function [target, flag] = planar_circle_tracker(C, P, radius, tolerance, rot, limit_pair)    
    %% Find nearest point on the circle perimeter closest to the point
    % https://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point
    pc = C - P; % point to center
    pc_norm_2 = sqrt(pc(1)^2 + pc(2)^2);
    np = C + (pc / pc_norm_2) * radius; % Nearest point on the circle
    
    %% Find nearest point is inside or outside of the circle
    pnp = np - p;
    % Case is inside of circle
    if pc_norm_2 < radius
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
        dot_num = pnp(1) * p_ctp(1) + pnp(2) * p_ctp(2);
        dot_den = norm(pnp) * norm(p_ctp);    

        angle = acos(dot_num / dot_den);
        
        % a x b since we need to know in comparison 
        % which is clockwise and anticlockwise
        if ctp_first
            sign_value =  p_ctp(1) * pnp(2) - p_ctp(2) * pnp(1);
        elseif ~ctp_first
            sign_value =  pnp(1) * p_ctp(2) - pnp(2) * p_ctp(1);
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

