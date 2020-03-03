function odoTargetPose = trans(transform,targetPose)
% odoTargetPose = trans(transform,targetPose)
% Transform a given point in world coordinates (targetPose) to odometry
% coordinates, using the origo of the odometry coordinates in world
% coordinates (transform).

    t_T = transform(3);
    t_o = targetPose(3) + t_T;
    p_o = [cos(t_T), -sin(t_T); sin(t_T), cos(t_T)]*targetPose(1:2) + transform(1:2);

    odoTargetPose = [p_o;t_o];
end