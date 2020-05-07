clear
clc
close all
%% select files

% k = 1; % choose the pair of files you want to compare

% set path to folder with files

pathToFolder = strcat(pwd, '/results');
direc = dir(pathToFolder);

% files in directory (omits "." and "..")
[numOfFiles, ~] = size(direc);
numOfFiles = numOfFiles - 2;
direc = direc(3:end);
files_2 = direc(1:numOfFiles/2);
files_1 = direc(numOfFiles/2+1:end);

% files_1 = ['raw/', direc(choice+numOfFiles/2).name];
% files_2 = ['raw/', direc(choice).name];

threshold_xy = 0.1;  % set threshold value

%% compute similarity
correctObjDetect = zeros(2, 4);
totalResultMatrix = zeros(1,4);


for k = 1:numOfFiles/2
    file1 = ['results/', files_1(k).name];
    file2 = ['results/', files_2(k).name];
    
    disp(file1);
    disp(file2);
    
    experiment = create_python_table(file1);
    test = table2array(readtable(file2));
    
    [measurements, ~] = size(test);
    [experiments, ~] = size(experiment);
    fprintf('Number of experiments NOT completed: %d\n\n', experiments-measurements);

    % matrix to store result: 0 = wrong, 1 = correct
    resultMatrix = zeros(measurements,4);
       
    for i =1:experiments
        object = experiment(i, 1);
        correctObjDetect(2, object) = correctObjDetect(2, object) + 1;
    end

    for i = 1:measurements
        %extract measurement data
        object = test(i,1);
        objx = test(i,2);
        objy = test(i,3);
        objPose = test(i,4);
        
        % skip if no object found
        if object == 0 ||object == -1
            continue
        end
        
        if object == experiment(i,1)
            resultMatrix(i,1) = 1;
            correctObjDetect(1, object) = correctObjDetect(1, object) + 1;
        end
        
        if abs(objx - experiment(i,2)) < threshold_xy
            resultMatrix(i,2) = 1;
        end
        
        if abs(objy - experiment(i,3)) < threshold_xy
            resultMatrix(i,3) = 1;
        end
        
        % check if correct pose
        if abs(objPose - experiment(i,4)) < 0.1
            resultMatrix(i,4) = 1;
        elseif (experiment(i,1) == 1 || experiment(i,1) == 2) 
            temp = abs(experiment(i,4) - objPose);

            % in the case of a square, check if measurement is 180
            % degrees apart
            if temp < 0.05 || abs(pi-temp) < 0.05
                resultMatrix(i,4) = 1;
            end
        end
    end

    totalResultMatrix = [resultMatrix; totalResultMatrix];

end

%% display result

disp(correctObjDetect)

% fully correct
allCorrect = sum(sum(totalResultMatrix,2)==4);

% correct object
objCorrect = sum(totalResultMatrix(:,1) == 1);

% correct x and y
xyCorrect = sum(sum(totalResultMatrix(:,2:3),2) == 2);

% correct pose
poseCorrect = sum(totalResultMatrix(:,4) == 1);

measurements = size(totalResultMatrix, 1) - 1;
fprintf('Out of %d completed experiments...\n', measurements);
fprintf('\t%d (%.2f %%) was fully correct\n', allCorrect, allCorrect/measurements*100);
fprintf('\t%d found the correct object\n', objCorrect);
fprintf('\t%d found the correct position\n', xyCorrect);
fprintf('\t%d found the correct pose\n', poseCorrect);


fprintf('For object 1 the accuracy was %.2f %%\n', correctObjDetect(1,1)/correctObjDetect(2,1)*100)
fprintf('For object 2 the accuracy was %.2f %%\n', correctObjDetect(1,2)/correctObjDetect(2,2)*100)
fprintf('For object 3 the accuracy was %.2f %%\n', correctObjDetect(1,3)/correctObjDetect(2,3)*100)
fprintf('For object 4 the accuracy was %.2f %%\n', correctObjDetect(1,4)/correctObjDetect(2,4)*100)
%% display actual result

x = [1 2 3, 4];
vals = correctObjDetect(1,:)./correctObjDetect(2,:)*100;
grid
b = bar(x,vals);
ylim([0, 105]);
xtips1 = b.XData;
ytips1 = round(b.YData, 1);
labels1 = strcat(string(ytips1), "%");
xlabel('Object number')
ylabel('Accuracy score %')
text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
    'VerticalAlignment','bottom')

%%
function A = create_python_table(filename)
    py = readtable(filename);
    [m, ~] = size(py);
    
    A = zeros(m,4);
    
    A(:,1) = py.Var1;
    A(:,4) = py.Var3;
    
%     py.Var2{1};

    new = strrep(py.Var2, ',', '');
    new = strrep(new,'[', '');
    new = strrep(new,']', '');

    for i = 1:m
        numstr = split(new{i});
        A(i,2) = str2double(numstr{1});
        A(i,3) = str2double(numstr{2});
    end
end
