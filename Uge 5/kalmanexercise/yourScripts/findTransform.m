function transform = findTransform(odoPose, pose)
% transform = FINDTRANSFORM(odoPose,pose)
% Find the transformation from the world coordinates to the odometry
% coordinates given a pose in the odometry coordinates (odoPose) and the
% same point in the world coordinates (pose). The output (transform) is
% simply the origo of the odometry coordinates in the world coordinates
    
    t_T = odoPose(3)-pose(3);
    
    A = [cos(t_T), -sin(t_T); sin(t_T), cos(t_T)];
    
    p_T = odoPose(1:2) - A*pose(1:2);
    
    transform = [p_T;t_T]; 
    
end