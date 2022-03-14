function M = getM(order)
    %Following the notation from
    %https://link.springer.com/article/10.1007/s003710050206 
    %("General matrix representations for B-splines")
    % See Theorem 1 of page 182

    % Uniform bspline
    % There are 2 ways, one is a shortcut, but the other has a real formulation
    % View the portion on (section 3.2) on NURBs and (section 4.1)

    % t(j) - t(j-1) = constant

    % j = start knot point
    % order = k-1 
    % m(i,j) = (1/(k-1)) * C(k-1-i,k-1) * ...
    %   sum(s=j to k-1) {pow(-1,s-j) * C(s-j,k) * pow(k-s-1,k-1-i)} 
    % C(i,n) = factorial(n)/(factorial(i) * factorial(n-i));
    k = order+1;
    
    M = zeros(k,k);

    for c = 1:numel(M)
        % notation is different for matlab
        if mod(c,k) == 0
            modv = k;
        else
            modv = mod(c,k);
        end
        idx = [ceil(c/k), modv];
        
        
        % because of matlab's notation we have step the element index down by 1
        i = idx(1) - 1; j = idx(2) - 1;
        C = @(i,n) factorial(n)/(factorial(i) * factorial(n-i));
        fac = 0;
        for s=j:(k-1)
            fac = fac + ((-1)^(s-j) * C(s-j,k) * (k-s-1)^(k-1-i));
        end
        m = (1/factorial(k - 1)) * C(k-1-i,k-1) * fac;
        
        M(ceil(c/k), modv) = m;
        %  fprintf("(Without matlab index) m(%d,%d) = %f\n",i-1,j-1,m);
        % fprintf("m(%d,%d) = %f\n",ceil(i/order),modv,M(ceil(c/order), modv));
    end
end

