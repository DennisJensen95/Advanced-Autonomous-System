function line = lsqLine(edges)
    % line = LSQLINE(edges) extract a line described in (alpha,r)
    % parameters from a set of edges
    
    n = length(edges);
    xmean = mean(edges(1,:));
    ymean = mean(edges(2,:));
    sumx = sum(edges(1,:));
    sumy = sum(edges(2,:));
    sumx2 = sum(edges(1,:).^2);
    sumy2 = sum(edges(2,:).^2);
    sumxy = sum(edges(1,:).*edges(2,:));
    
    alpha = 1/2*atan2((2*sumx*sumy-2*n*sumxy),(sumx^2-sumy^2-n*sumx2+n*sumy2));
    
    r = xmean*cos(alpha) + ymean*sin(alpha);
    
    if r<0
        r = abs(r);
        if alpha<0
            alpha = alpha + pi;
        else
            alpha = alpha - pi;
        end
    end
    
    line = [alpha,r];
end