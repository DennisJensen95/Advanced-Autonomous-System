clear
clc
close all
%% init

object = 1; %set object 

point = [2.2836633; 1.72343886];
pose = 1.7145883072696777;

%% calc corners

R = [cos(pose),-sin(pose);sin(pose),cos(pose)];

if object == 1
    L = 0.40;
    b = 0.15;
    
    point1 = [point(1)+1/2*L; point(2)+1/2*b];
    point2 = [point(1)-1/2*L; point(2)+1/2*b];
    point3 = [point(1)-1/2*L; point(2)-1/2*b];
    point4 = [point(1)+1/2*L; point(2)-1/2*b];

    points = [point1,point2,point3,point4];
elseif object == 2
    L = 0.30;
    b = 0.20;
    
    point1 = [point(1)+1/2*L; point(2)+1/2*b];
    point2 = [point(1)-1/2*L; point(2)+1/2*b];
    point3 = [point(1)-1/2*L; point(2)-1/2*b];
    point4 = [point(1)+1/2*L; point(2)-1/2*b];

    points = [point1,point2,point3,point4];
elseif object == 3
    L = 0.40;
    b = 0.10;
    
    point1 = [point(1)+L; point(2)];
    point2 = [point(1); point(2)+b];

    points = [point, point1,point2];
elseif object == 4
    L = 0.30;
    b = 0.15;
    
    point1 = [point(1)+L; point(2)];
    point2 = [point(1); point(2)+b];

    points = [point, point1,point2];
end


s = points-point; %subtract center
so = R*s; %rotate
vo = so+point; %add center

%% plot
close all
% vo = [1.708117 1.610521
% 2.009148 1.587979 
% 1.999827 1.436915 
% 2.004409 1.511346 
% 0.075676 0.226613 ]';

% vo = [1.78663 1.9398                                                                                                                                                                       
% 1.87128 1.81452                                                                                                                                                                      
% 1.45442 1.71567                                                                                                                                                                      
% 1.53861 1.59065                                                                                                                                                                      
% 1.97882 1.88688]';
% 
% vo = [1.78663 1.9398                                                                                                                                                     
% 1.87137 1.81438                                                                                                                                                    
% 1.45442 1.71567                                                                                                                                                    
% 1.53898 1.59009                                                                                                                                                    
% 0.15138 0.400867 ]';

figure(1)
plot(vo(1,:),vo(2,:), '*r')
grid
axis([1,3,1,2])
axis equal
hold on
g = rectangle('Position', [1,1,2,1]);
    g.EdgeColor = [0,0.6,0];
    g.LineWidth = 1.5;
    g.LineStyle = '--';

vo