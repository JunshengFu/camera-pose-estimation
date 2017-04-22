close all
clc
clear

%% set folder path
posePath     = './testData/pose/00.txt';
lidarPath    = './testData/lidar/';
imagePath    = './testData/image/';
calibPath    = './testData/calibration/calib.txt';
rootPath = struct('posePath', posePath, 'lidarPath', lidarPath, 'imagePath'...
    , imagePath, 'calibPath', calibPath);

%% parameter setup
method = 'knn';             % 1: KNN
threshold = 3;              % in order to select the depth vaule for query pixel, threshold
                            % is maximum distance (in pixel) between the reference pixel 
                            % (with depth) and the query pixel
featureType = 'surf';       % feature types: surf, mser, minEigen , fast, harris                         
visulization_flag = true;   % visulize the estimated camera pose  

%% give index of query and reference images, then compute query's camera pose
image_idx = 3823;
ref_idx = 3814;

[D, err_translation, err_angles, err_max_angle, timesList]...
    = poseFeatureBased(rootPath, image_idx, ref_idx, method, threshold, visulization_flag, featureType);


%% print result
fprintf('Translation err in meter: %f m\n', err_translation);
fprintf('Max rotation err in degree: %f degree\n', err_max_angle);
fprintf('Distance between query and reference images: %f meters \n', D);
fprintf('Processing time: %f s\n', timesList);
