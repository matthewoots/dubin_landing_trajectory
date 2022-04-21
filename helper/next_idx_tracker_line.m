function next_idx = next_idx_tracker_line(idx, P, tolerance, wp)
    
    angle = 0;
    next_idx = idx - 1;
    
    while angle < tolerance
        next_idx = next_idx + 1;
        start_p = wp(next_idx-1,:);
        curr_track_p = wp(next_idx,:);
        
        % https://www.nagwa.com/en/explainers/939127418581/  
        % we find the vector (d) between prev and next idx
        vector_d = curr_track_p - start_p;
        mag_d = sqrt(vector_d(1)^2 + vector_d(2)^2 + vector_d(3)^2);
        d = vector_d / mag_d;
        
        % https://stackoverflow.com/questions/1811549/perpendicular-on-a-line-from-a-given-point
        % translate the point and get the dot product
        lambda = (d(1) * (P(1) - start_p(1))) + ...
            (d(2) * (P(2) - start_p(2))) + ...
            (d(3) * (P(3) - start_p(3)));
        inters = (d * lambda) + start_p;    

        % Check whether point C is on the left or right
        % Use sign of the determinant of vectors 
        % (AB,AM), where M(X,Y) is query point

        det = vector_d(1) * (inters(2) - start(2)) - ...
            vector_d(2) * (inters(1) - start(1));

        if det == 0
           sign = 0;
           ctp_first = false;
        else
           sign = det / abs(det);
           if sign < 0
               ctp_first = true;
           elseif sign > 0
               ctp_first = false;
           end
        end
        % 0 is on the line, +1 is right of line, -1 is left of line
        % If on the right we use ctp_last and if left we use ctp_first
        p_np = P - inters;
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

