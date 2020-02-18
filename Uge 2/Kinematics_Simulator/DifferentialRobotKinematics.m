clear
clc
close all
%% Initialization
global pose %current pose
global ts %sampling time
global wheelspeed % [angular velocity right wheel, angular velocity left wheel]
global robotpar % [wheel separation, radius right wheel,radius left wheel]
global pose_vec %pose log
global kvals % [k_rho, k_alpha, k_beta]


pose = [0;0;0];
ts = 0.01;
wheelspeed = [0,0];
robotpar = [0.26, 0.035, 0.035];
pose_vec = [0;0;0];
kvals = [0.3,0.8,-0.15];


DriveOption = 4; %1: square, 2: hexagon, 3: star, 4: neither

%% Perform action

if DriveOption == 1
    SquareDrive();
    PrintDrive(1.5);
elseif DriveOption == 2
    HexaDrive();
    PrintDrive(2)
elseif DriveOption == 3
    StarDrive();
    PrintDrive(3)
end
    
input = [0;0;pi/2];
Move2PoseController(input)
PrintDrive(0.7);


% Move2PoseController(input);



%% functions

function KinUpdate()
    global pose
    global pose_vec
    
    pose = pose + DiffKinematics();
    pose(3) = atan2(sin(pose(3)),cos(pose(3)));
    pose_vec = [pose_vec, pose];
end

function poseUpdate = DiffKinematics()
    global pose
    global ts
    global wheelspeed
    global robotpar

    theta = pose(3);
    vr = wheelspeed(1);
    vl = wheelspeed(2);
    w = robotpar(1);
    rr = robotpar(2);
    rl = robotpar(3);

    poseUpdate = zeros(3,1);

    poseUpdate(1) = (cos(theta)*(rl*vl + rr*vr))/2*ts;
    poseUpdate(2) = (sin(theta)*(rl*vl + rr*vr))/2*ts;
    poseUpdate(3) = -(-rl*vl + rr*vr)/w*ts;
end

function GoForward(distance, speed)
    global pose
    global wheelspeed
    global robotpar

    wheelspeed = [speed/robotpar(2), speed/robotpar(3)];

    startpose = pose;
    dist_travel = 0;
    while dist_travel < distance
        KinUpdate();    
        dist_travel = norm(startpose(1:2) - pose(1:2));
    end
end

function Turn(angle, speed)
    global wheelspeed
    global robotpar
    global pose
    
    if angle > 0
        wheelspeed(1) = speed/robotpar(2);
        wheelspeed(2) = -speed/robotpar(3);
    else
        wheelspeed(1) = -speed/robotpar(2);
        wheelspeed(2) = speed/robotpar(3);
    end
    
    start_angle = pose(3);
    turned_angle = 0;
    while abs(turned_angle) < abs(angle)
        KinUpdate();
        turned_angle = pi - abs(abs(pose(3) - start_angle) - pi);
    end

end

function PrintDrive(ax)
    global pose_vec

    figure(1)
    plot(pose_vec(1,:),pose_vec(2,:), '-r', 'LineWidth', 3)
    hold on
    plot(pose_vec(1,end),pose_vec(2,end), 'or', 'LineWidth', 3)
    quiver(pose_vec(1,end),pose_vec(2,end), cos(pose_vec(3,end))/3, sin(pose_vec(3,end))/3, 'LineWidth', 3,'Color','k','MaxHeadSize',0.8)
    grid on
    title('Robot movement')
    xlabel('x [m]')
    ylabel('y [m]')
    axis([-ax,ax,-ax,ax])
end

function PrintTheta()
    global pose_vec
    global ts
    
    time = linspace(0,(length(pose_vec)-1)*ts,length(pose_vec));
    
    figure(2)
    plot(time,pose_vec(3,:)*180/pi,'r','LineWidth',3)
    grid on
    title('Robot heading')
    ylabel('Heading [degree]')
    xlabel('Time [s]')
    
end

function Move2Pose(input, vel, turn_vel)
    global pose
    
    target = input - pose;
    
    rho = norm(target(1:2));
    alpha = -pose(3) + atan2(target(2),target(1));
    beta = target(3)-alpha;
    
    Turn(alpha,turn_vel);
    GoForward(rho,vel);
    Turn(beta,turn_vel);    
end

function Move2PoseController(input)
    global pose
    global kvals
    global wheelspeed
    global robotpar
    
    target = input - pose;
    thTw = target(3);
    M = [cos(thTw), sin(thTw), 0;
            -sin(thTw), cos(thTw), 0;
            0,0,1];
    
    k = 0;
    while true     
        xx = pose - target;
        poseT = M*xx;
        xT = poseT(1);
        yT = poseT(2);
        
        rho = sqrt(xT^2 + yT^2);
        alpha = -poseT(3) + atan2(-yT,-xT);
        alpha = atan2(sin(alpha),cos(alpha));
        beta = -poseT(3) - alpha;
        beta = atan2(sin(beta),cos(beta));
        
        v = kvals(1)*rho;
        omega = kvals(2)*alpha + kvals(3)*beta;
        
        if v < 0.005 && omega < 0.005
            fprintf('Speed too low. Ending simulation...\n');
            break;
        end
       
        wheelspeed(1) = (2*v-omega*robotpar(1))/(2*robotpar(2));
        wheelspeed(2) = (2*v+omega*robotpar(1))/(2*robotpar(3));
        
        d_error = norm(pose(1:2)-target(1:2));
        a_error = pi - abs(abs(pose(3) - thTw) - pi);
        
        if d_error < 0.01 && a_error < 0.02
            fprintf('Destination reached\n');
            break;
        else
            KinUpdate();
        end
        
        if k > 5000
            fprintf('Maximum iterations reached\n');
            break
        end
        k = k + 1;
    end
    
    
    %         if alpha > -pi/2 && alpha <= pi/2
%             sys = [-cos(alpha), 0; 
%                 sin(alpha)/rho, -1; 
%                 -sin(alpha)/rho, 0];
%         else
%             sys = [cos(alpha), 0; 
%                 -sin(alpha)/rho, -1; 
%                 sin(alpha)/rho, 0];
%         end
%         yy = sys*[v;omega];
%         
%         rhod = yy(1);
%         alpd = yy(2);
%         betd = yy(3);
    
end

function SquareDrive()
    GoForward(1, 2)
    Turn(pi / 2, 1)
    GoForward(1, 3)
    Turn(pi / 2, 1)
    GoForward(1, 3)
    Turn(pi / 2, 1)
    GoForward(1, 3)
    Turn(pi / 2, 1)
end

function HexaDrive()
    GoForward(1, 2)
    Turn(pi / 3, 1)
    GoForward(1, 3)
    Turn(pi / 3, 1)
    GoForward(1, 3)
    Turn(pi / 3, 1)
    GoForward(1, 3)
    Turn(pi / 3, 1)
    GoForward(1, 3)
    Turn(pi / 3, 1)
    GoForward(1, 3)
    Turn(pi / 3, 1)
end

function StarDrive()
    GoForward(1,1)
    Turn(108*pi/180,0.1)
    GoForward(1,1)
    Turn(-36*pi/180,0.1)
    GoForward(1,1)
    Turn(108*pi/180,0.1)
    GoForward(1,1)
    Turn(-36*pi/180,0.1)
    GoForward(1,1)
    Turn(108*pi/180,0.1)
    GoForward(1,1)
    Turn(-36*pi/180,0.1)
    GoForward(1,1)
    Turn(108*pi/180,0.1)
    GoForward(1,1)
    Turn(-36*pi/180,0.1)
    GoForward(1,1)
    Turn(108*pi/180,0.1)
    GoForward(1,1)
end