clear
clc
close all
%% initialize

folder = 'FILEPATH';
addpath(folder);

name_laser = 'laser_0.log';
laser = load(name_laser); % load laser data
name_odo = 'odoPose.log';
odo = load(name_odo);


%% load laser data

timeL       =   laser(:,1);
N_L         =   length(timeL);
scanNum     =   laser(:,2);
dangle      =   laser(1,3);
firstAngle  =   laser(1,4);
nmeas       =   laser(1,5);
range   =   laser(:,6:end);
        
% calculate x,y coordinates for laser measurements in laser scanner
% coordinate system
for i = 1:N_L
    x_laser(i,:) = cosd(firstAngle+dangle*(0:(nmeas-1))).*range(i,:);
    y_laser(i,:) = sind(firstAngle+dangle*(0:(nmeas-1))).*range(i,:);
end

%% load odo data
timeO       =   odo(:,1);
x_odo       =   odo(:,2);
y_odo       =   odo(:,3);
th_odo      =   odo(:,4);
vel         =   odo(:,5);
flag        =   odo(:,6);

timelapse = timeO-timeO(1);

%Remove lines in the odoPose.log until the x-coordinate changes
idx = find(diff(x_odo) > 0, 1, 'first');
x_odo = x_odo(idx:end);


%% plot laser data
UnixToDate(timeL(1));
UnixToDate(timeO(1));

i = 10; %one second

xFilter = x_laser(i,:);
yFilter = y_laser(i,:);

figure(1)
plot(xFilter,yFilter, '*r','LineWidth', 3)
xlabel('x [m]')
ylabel('y [m]')
grid
axis([0,4,-2,2]);

%% plot odo data
figure(2)
plot(x_odo,y_odo, '-r', 'LineWidth',3)
xlabel('x [m]')
ylabel('y [m]')
grid
axis([0,4,-2,2]);