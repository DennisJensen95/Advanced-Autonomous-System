%% exercise 7
clear
clc
close all

%% problem 1

lines = [3;-4;3;1]; %start x,y ; end x,y
pose_l = [0,0,0];

scan = laserscan2011(pose_l(1),pose_l(2),pose_l(3),lines,4,0.36,180);
figure(1)
subplot(2,1,1)
polarplot(scan(1,:),scan(2,:),'r')
grid on

%% problem 2

carScan = polar2carth(scan);

figure(1)
subplot(2,1,2)
plot(carScan(1,:),carScan(2,:))
grid on
xlabel('x')
ylabel('y')

%% problem 3

carScanNew = transform(pose_l,carScan);

%% problem 4
lines1 = [0;1;2.5;1];
lines2 = [2.5;1;2.5;5];
lines3 = [0;-1;3.5;-1];
lines4 = [3.5;-1;3.5;5];
lines = [lines1,lines2,lines3,lines4];

pose_lx = [0,1,1.5];
pose_ly = [0,0,0];
pose_lz = [0,0,pi/6];
poses = [pose_lx; pose_ly; pose_lz];

figure(2)
hold on
colStr = ['r','b','k'];
width = [3,2,2];
for i = 1:3
    scan(:,:,i) = laserscan2011(pose_lx(i),pose_ly(i),pose_lz(i),lines,4,0.36,180);
    carScan(:,:,i) = polar2carth(scan(:,:,i));
    carScanNew(:,:,i) = transform([pose_lx(i),pose_ly(i),pose_lz(i)],carScan(:,:,i));
    plot(carScanNew(1,:,i),carScanNew(2,:,i),colStr(i),'LineWidth',width(i))
end

ylim([-1,3])
axis equal
grid on
xlabel('x')
ylabel('y')


figure(2)
for i = 1:3
    plot(pose_lx(i),pose_ly(i),strcat(colStr(i),'x'))
    quiver(pose_lx(i),pose_ly(i),cos(pose_lz(i))*0.5,sin(pose_lz(i))*0.5,colStr(i),'MaxHeadSize', 1)
end
for i = 1:length(lines(1,:))
    line([lines(1,i),lines(3,i)],[lines(2,i),lines(4,i)], 'Color', 'green','LineWidth',1)
    hold on
end
legend('Pose 1', 'Pose 2', 'Pose 3','Wall')

clear lines1 lines2 lines3 lines4 colStr width pose_lx pose_ly pose_lz pose_l

%% problem 5

y = linspace(2,5,10);
% y = -x-3;
x = ones(1,length(y))*2.5;

% figure(1)
% plot(x,y,'ro')
% grid on
% axis([-5,5,-5,5])

points = [x;y];

polLine = lsqline(points);

carLine = polar2carth(polLine');

fprintf('Line parameter (alpha,r): \t(%.2f,%.2f)\n', polLine(1),polLine(2));
fprintf('Line parameter (x,y): \t\t(%.2f,%.2f)\n\n', carLine(1),carLine(2));

clear x y points polLine carLine

%% problem 6

% figure(1)
% plot(carScan(1,:,1),carScan(2,:,1),'r','LineWidth',3)
% grid on
% xlabel('x')
% ylabel('y')

x1 = linspace(0,2.5,10);
y1 = ones(1,length(x1));

x2 = linspace(0,3.5,10);
y2 = -ones(1,length(x2));

y3 = linspace(-1,1.5,10);
x3 = ones(1,length(y3))*3.5;


%find line parameters
linepar1 = lsqline([x1;y1]);
linepar2 = lsqline([x2;y2]);
linepar3 = lsqline([x3;y3]);

fprintf('Line parameters -> scan 1 (alpha, r):\n\t1: (%.2f, %.2f)\n\t2: (%.2f, %.2f)\n\t3: (%.2f, %.2f)\n\n',...
    linepar1(1),linepar1(2),linepar2(1),linepar2(2),linepar3(1),linepar3(2))

%% problem 7

% figure(1)
% plot(carScan(1,:,3),carScan(2,:,3),'r','LineWidth',3)
% grid on
% xlabel('x')
% ylabel('y')


coefficients = polyfit([0, 1.223], [-1.1547, -1.8643], 1);
a(1) = coefficients(1);
b(1) = coefficients(2);

coefficients = polyfit([1.233, 2.7237], [-1.8643, 0.7176], 1);
a(2) = coefficients(1);
b(2) = coefficients(2);

coefficients = polyfit([0, 1.3624], [1.1547, 0.36812], 1);
a(3) = coefficients(1);
b(3) = coefficients(2);

x1 = linspace(0,1.223,10);
y1 = a(1)*x1+b(1);

x2 = linspace(0,1.3624,10);
y2 = a(2)*x2+b(2);

x3 = linspace(1.233,2.7237,10);
y3 = a(3)*x3+b(3);


%find line parameters
linepar1 = lsqline([x1;y1]);
linepar2 = lsqline([x2;y2]);
linepar3 = lsqline([x3;y3]);

fprintf('Line parameters -> scan 3 (alpha, r):\n\t1: (%.2f, %.2f)\n\t2: (%.2f, %.2f)\n\t3: (%.2f, %.2f)\n\n',...
    linepar1(1),linepar1(2),linepar2(1),linepar2(2),linepar3(1),linepar3(2))

clear x1 x2 x3 y1 y2 y3 a b coefficients

%% problem 8

pose = poses(:,3);

linepar1new = transline(linepar1,pose);
linepar2new = transline(linepar2,pose);
linepar3new = transline(linepar3,pose);

fprintf('Converting scan 3 to world coordinates...\n');
fprintf('Line parameters -> scan 3 (alpha, r):\n\t1: (%.2f, %.2f)\n\t2: (%.2f, %.2f)\n\t3: (%.2f, %.2f)\n\n',...
    linepar1new(1),linepar1new(2),linepar2new(1),linepar2new(2),linepar3new(1),linepar3new(2))


%% functions
function coord = polar2carth(pol)
    coord = [(cos(pol(1,:)).*pol(2,:));
             (sin(pol(1,:)).*pol(2,:))];
end

function coord = transform(systempose_w, pos_l)
    th_lw = systempose_w(3);
    
    A = [cos(th_lw), -sin(th_lw); sin(th_lw), cos(th_lw)];
    
    coord = A*pos_l + systempose_w(1:2)';
end

function line=lsqline(points)
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

function lineparW = transline(lineparL, poseL)
    aL = lineparL(1);
    rL = lineparL(2);
    
    xL = poseL(1);
    yL = poseL(2);
    thL = poseL(3);
    
    aW = aL + thL;
    rW = rL + [cos(aW),sin(aW)]*[xL,yL]';
    
    lineparW = [aW, rW];
end