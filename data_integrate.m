%-----------------------------------------%
%--- Integrate robot state-input pairs ---%
%-----------------------------------------%
%% DP
classname = ['DP'];

data_collection = {};
nump = 1;
numq = 1;
for i = 0 : 1 : 59
    data_load = load(['data\DP\',num2str(i),'.txt']);
    t = data_load(:, end);
    x = data_load(:, 2 : 3);
    u = data_load(:, 1);
    y = x;

    % create validation and training sets
    if mod(i + 1, 4) == 0
        data_collection.val{1, nump}.t = t;
        data_collection.val{1, nump}.x = x;
        data_collection.val{1, nump}.y = y;
        data_collection.val{1, nump}.u = u;
        nump = nump + 1;
    else
        data_collection.train{1, numq}.t = t;
        data_collection.train{1, numq}.x = x;
        data_collection.train{1, numq}.y = y;
        data_collection.train{1, numq}.u = u;
        numq = numq + 1;
    end
end
fname = ['data' , filesep , classname, '.mat'];
save(fname , 'data_collection');

%% TDVPT
classname = ['TDVPT'];

data_collection = {};
nump = 1;
numq = 1;
for i = 0 : 1 : 161
    data_load = load(['data\TDVPT\', num2str(i), '.txt']);
    t = [];
    t1 = data_load(:, 5);
    x = data_load(:, 1 : 2);
    u = data_load(:, 3 : 4);
    y = x;
    t = zeros(length(t1), 1);
    for j =1 :length(t1)
        t(j, 1) = sum(t1(1 : j, 1));
    end
    
    % create validation and training sets
    if mod(i + 1, 4) == 0
        data_collection.val{1, nump}.t = t;
        data_collection.val{1, nump}.x = x;
        data_collection.val{1, nump}.y = y;
        data_collection.val{1, nump}.u = u;
        nump = nump + 1;
    else
        data_collection.train{1, numq}.t = t;
        data_collection.train{1, numq}.x = x;
        data_collection.train{1, numq}.y = y;
        data_collection.train{1, numq}.u = u;
        numq = numq + 1;
    end
end
fname = ['data', filesep, classname, '.mat'];
save(fname , 'data_collection');

%% Soft Robot
classname = ['SoftRobot'];

data = load( ['data\SoftRobot\' , 'softrobot.mat'] );
data_collection.val = data.val;
data_collection.train = data.train;
fname = ['data', filesep, classname, '.mat'];
save(fname , 'data_collection');

%% AUV
classname = ['AUV'];

data_collection = {};
nump = 1;
numq = 1;
for i = 0 : 1 : 59
    data_load = load(['data\AUV\', num2str(i), '.txt']);
    t = data_load(:, end);
    x = data_load(:, 3 : 8);
    u = data_load(:, 1 : 2);
    y = x;
    
    % create validation and training sets
    if mod(i + 1, 6) == 0
        data_collection.val{1, nump}.t = t;
        data_collection.val{1, nump}.x = x;
        data_collection.val{1, nump}.y = y;
        data_collection.val{1, nump}.u = u;
        nump = nump + 1;
    else
        data_collection.train{1, numq}.t = t;
        data_collection.train{1, numq}.x = x;
        data_collection.train{1, numq}.y = y;
        data_collection.train{1, numq}.u = u;
        numq = numq + 1;
    end
end
fname = ['data', filesep, classname, '.mat'];
save(fname , 'data_collection');

%% AUV for 3D task
classname = ['AUV_3D'];

data_collection = {};
nump = 1;
numq = 1;
for i = 0 : 1 : 59
    data_load = load(['data\AUV_3D\', num2str(i), '.txt']);
    t = data_load(:, end);
    x = data_load(:, 4 : 15);
    u = data_load(:, 1 : 3);
    y = x;
    
    % create validation and training sets
    if mod(i + 1, 6) == 0
        data_collection.val{1, nump}.t = t;
        data_collection.val{1, nump}.x = x;
        data_collection.val{1, nump}.y = y;
        data_collection.val{1, nump}.u = u;
        nump = nump + 1;
    else
        data_collection.train{1, numq}.t = t;
        data_collection.train{1, numq}.x = x;
        data_collection.train{1, numq}.y = y;
        data_collection.train{1, numq}.u = u;
        numq = numq + 1;
    end
end
fname = ['data', filesep, classname, '.mat'];
save(fname , 'data_collection');

%% Renguin-Insipred Robot
classname = ['RoboticPenguin'];

data_collection = {};
for i = 0 : 1 : 8
    data_load = load(['data\RoboticPenguin\', num2str(i), '.txt']);
    t = data_load(:, 8);
    x = data_load(:, 4 : 6);
    u = data_load(:, 7);
    y = x;
    
    % create validation and training sets
    data_collection.val{1, i + 1}.t = t;
    data_collection.val{1, i + 1}.x = x;
    data_collection.val{1, i + 1}.y = y;
    data_collection.val{1, i + 1}.u = u;
    data_collection.train{1, i + 1}.t = t;
    data_collection.train{1, i + 1}.x = x;
    data_collection.train{1, i + 1}.y = y;
    data_collection.train{1, i + 1}.u = u;
end
fname = ['data', filesep, classname, '.mat'];
save(fname , 'data_collection');

%% Renguin-Insipred Robot with Location Information
classname = ['RoboticPenguin_full'];

data_collection = {};
for i = 0 : 1 : 8
    data_load = load(['data\RoboticPenguin\', num2str(i), '.txt']);
    t = data_load(:, 8);
    x = data_load(:, 1 : 6);
    u = data_load(:, 7);
    y = x;
    
    % create validation and training sets
    data_collection.val{1, i + 1}.t = t;
    data_collection.val{1, i + 1}.x = x;
    data_collection.val{1, i + 1}.y = y;
    data_collection.val{1, i + 1}.u = u;
    data_collection.train{1, i + 1}.t = t;
    data_collection.train{1, i + 1}.x = x;
    data_collection.train{1, i + 1}.y = y;
    data_collection.train{1, i + 1}.u = u;
end
fname = ['data', filesep, classname, '.mat'];
save(fname , 'data_collection');
