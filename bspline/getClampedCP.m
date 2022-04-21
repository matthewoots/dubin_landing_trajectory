function cp = getClampedCP(path,order)
    clampstart = []; clampend = [];
    for i = 1:order-1
        clampstart = [clampstart path(1)];
        clampend = [clampend path(end)];
    end
    cp = [clampstart path clampend];
end

