function [pos, vel, tt] = getBSpline(order, timespan, ctrlpt, knotdiv, isclamped)
    %% Using Bspline Segment code

    % Following the notation from
    % (1) https://link.springer.com/article/10.1007/s003710050206 
    % ("General matrix representations for B-splines")

    % Following the notation from
    % (2) (Real-Time Trajectory Replanning for MAVs using Uniform B-splines 
    % and a 3D Circular Buffer)

    % According to (2) etc quintic bspline definition of u is
    % s_t = (time - t(1)) / delta_t
    % u(t) = s(t) − s(i)
    % s(i) lies at the time of the control point(i)

    % According to (2) etc quintic bspline uses a knot vector of
    % [t(i−2), t(i−1), t(i), t(i+1), t(i+2), t(i+3)]

    % p (control points) and t (times allocated to each control point) has a
    % range of 0 to n (which is i = 0 to n)

    % p(t) lies inside [t(i), t(i+1)), and they depend on k (order+1) control points

    k = order + 1;
    n = numel(ctrlpt) - 1;

    M = getM(order);
    u = zeros(1,k); du = zeros(1,k); ddu = zeros(1,k); dddu = zeros(1,k);
    p = zeros(k,1);

    % p = degree
    % n = cp - 1
    % knots = m + 1
    % basics is m = n + p + 1
    % Open B-splines are [knot(p), knot(m-p)]
    % * Clamping only happens for the knots not for the control points!
    % * Control points are not really attached to time

    %% User inputs
    
    P = ctrlpt;
    % How much to segment a knotspan (internal evaluation in a spline segment)
    % "isclamped" applies here
    % Range of index to evaluate according
    range = [order:numel(P)]; 


    %% Main code
    % I may have made the code abit different by not using knots = m + 1 which
    % means using numel(P) rather than knots_total to segment the time horizon
    % Solution for matching time span
    % dt = (timespan(2) - timespan(1)) / (numel(range)-1); % span of 1 knot
    % t = linspace(timespan(1), timespan(2), (numel(range))); % knots
    
    dt = (timespan(2) - timespan(1)) / (numel(range)-(order+1)); % span of 1 knot
    t = linspace(timespan(1), timespan(2), numel(range)); % knots
    % End of solution for matching time span
    
    pos = []; vel = []; acc = []; jrk = []; tt = []; 
    
    kn_seg = knotdiv; % Division of 1 span or 1 segment
   
    for l = 1:numel(range)-1
    % for l = 1:numel(range)-1-order
        idx = range(l) - order;
        % fprintf('idx %f/%f\n', idx, numel(range));
   
        nxt_idx = idx + 1; 
        lpos = zeros(1,kn_seg-1);
        lvel = zeros(1,kn_seg-1);
        lacc = zeros(1,kn_seg-1);

        span = linspace(idx, nxt_idx, kn_seg); % relative to the start time as 0 regardless of the time 
        actualspan = linspace(t(l), t(l+1), kn_seg+1); % time in abs (simulation time / given time)
        % fprintf("Compare span %f / %f\n",dt/kn_seg, actualspan(2)-actualspan(1));
        
        if idx < 0
            error('idx is below suggested idx for control points');
        end 

        if idx + 1>= numel(P)
            error('idx is out of bounds compared to control points');
        end

        tic
        % numel(span)-1 becasue we are considering [0,1) hence not including 1
        for m = 1:numel(span)-1

            % Using convention from (1) to get u
            time = span(m); % current time in index form, of course we dont like to play with conversion
            u_t = (time - idx)/((idx+1) - idx); % using index is the same as using time since u_t is a factor
            % Save the time constant (0 to 1) and the actual time of the point
            timeConst(m) = u_t;
            timeActual(m) = actualspan(m);

            % p have several conventions according to SO many papers but i
            % would use the convention under (1) and it is the correct one
            % etc if order is 5
            % (1) p = [P(idx-5) P(idx-4) P(idx-3) P(idx-2) P(idx-1) P(idx)]';

            % Make the u, du, ddu and p matrix
            for j = 1:k
                u(j) = u_t^(j-1);
                p(j) = P(idx + (j-1) + 1); % we add a +1 here for matlab notation
                if j >= 2
                    du(j) = (j-1) * u_t^(j-2);
                end
                if j >= 3
                    ddu(j) = (j-1) * (j-2) * u_t^(j-3);
                end
                if j >= 4
                    dddu(j) = (j-1) * (j-2) * (j-3) * u_t^(j-4);
                end
            end

            inv_dt = 1/dt;

            % Matrix multiplication to attain the pos, vel and acc
            lpos(m) = u * M * p;
            lvel(m) = inv_dt * du * M * p;
            % lacc(m) = inv_dt^2 * ddu * M * p;
            % ljrk(m) = inv_dt^3 * dddu * M * p;
        end

        % Add the segment to the plot array
        pos = [pos lpos];
        vel = [vel lvel];
        % acc = [acc lacc];
        % jrk = [jrk ljrk];
        tt = [tt timeActual];
        % tc = [tc timeConst];
    end
    
end