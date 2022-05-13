function m = getSingleM(index, order)
    %Following the notation from
    %https://link.springer.com/article/10.1007/s003710050206 
    %("General matrix representations for B-splines"
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
    % because of matlab's notation we have step the element index down by 1
    i = index(1) - 1; j = index(2) - 1;
    C = @(i,n) factorial(n)/(factorial(i) * factorial(n-i));
    fac = 0;
    for s=j:(k-1)
        fac = fac + ((-1)^(s-j) * C(s-j,k) * (k-s-1)^(k-1-i));
    end
    m = (1/factorial(k - 1)) * C(k-1-i,k-1) * fac;

    %  fprintf("(Without matlab index) m(%d,%d) = %f\n",i-1,j-1,m);
end

