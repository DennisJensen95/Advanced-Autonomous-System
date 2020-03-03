function line=lsqLine(points)
    n = length(points);
    xmean = mean(points(1,:));
    ymean = mean(points(2,:));
    sumx = sum(points(1,:));
    sumy = sum(points(2,:));
    sumx2 = points(1,:)*points(1,:)';
    sumy2 = points(2,:)*points(2,:)';
    sumxy = points(1,:)*points(2,:)';
    
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