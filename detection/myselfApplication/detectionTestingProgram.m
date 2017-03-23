%% initialize need directory
startTime0 = clock;

path_detectionUtils = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\deepGraspingCode1\detection\detectionUtils';
path_utils = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\deepGraspingCode1\util';
addpath(sprintf(path_detectionUtils,pwd));
addpath(sprintf(path_utils,pwd));

dataDir = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\rawDataSet';
bgrDir = 'E:\WORK\ORGANIZATION\NTUT\Robot Grasping\Project02\Code\rawDataSet';
instNum = sscanf('pcd0107r.png', '%*3c%u')

%% load need data for detection
load ../../data/bgNums.mat
load ../../data/graspModes24.mat
load ../../weights/graspWFinal.mat
load ../../data/graspWhtParams.mat

%% config parameters for detection
% Just use the positive-class weights for grasping.
w_class = w_class(:,1);

bgFN = sprintf('%s/pcdb%04dr.png',bgrDir,bgNo(instNum));

rotAngs = 0:15:(11*15);
heights = 10:10:90;
widths = 10:10:90;
scanStep = 10;

%% detect
[bestRects,bestScores] = onePassDectionForInstDefaultParamsDisplay(instNum,dataDir,bgrDir)

elapsedTim0 = etime(clock, startTime0)