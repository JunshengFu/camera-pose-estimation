function [D, err_translation, err_angles, err_max_angle, timesList]...
    = poseFeatureBased(rootPath, idx_qury, idx_ref, method, threshold, plot_flag, featureType)
%EVALUATEKIMAGES Summary of this function goes here
%   Detailed explanation goes here
% Input:
%  rootPath: a sturcture constist of all related file path
%  N: number of all images
%  method: methods used for find the depth value
%  threshold: in order to select the depth vaule for query pixel, threshold
%  is maximum distance (in pixel) between the reference pixel (with depth) and the query pixel
%  plot_flag; plot (true) the figure or not (false)
%  featureType: feature used finding the matches between query and reference images.
%
% Output:
%  D: distance array corresponding to the selected images
%  image_idx: index array of selected images
%  err_translation: an array of translation error(in meter)  
%  err_angles: an array of rotation errors (in degree)  
%  err_max_angle: an array of max rotation error (in degree) 
%  timesList: array of times that each image is poressed

tic
 
%% define the objects
s1_lidar_flag = true;  % reference image has the lidar
s2_lidar_flag = false; % query image doesen't have lidar

s1 = ref(rootPath, idx_ref, s1_lidar_flag); % reference obj
I1 = s1.image;
s2 = ref(rootPath, idx_qury, s2_lidar_flag); % query obj
I2 = s2.image;

% get intrinsic matrix
K = s2.K(1:3,1:3);
K = K';
cameraParams = cameraParameters('IntrinsicMatrix',K);


%% detect and match features 
if size(I1,3) == 3
    grayImage_1 = rgb2gray(I1);
    grayImage_2 = rgb2gray(I2);    
else
    grayImage_1 = I1;
    grayImage_2 = I2;    
end

% parameter
switch featureType
    case 'surf'
        points_1 = detectSURFFeatures(grayImage_1);
        points_2 = detectSURFFeatures(grayImage_2);    
    case 'mser'
        points_1 = detectMSERFeatures(grayImage_1);
        points_2 = detectMSERFeatures(grayImage_2); 
    case 'minEigen'
        points_1 = detectMinEigenFeatures(grayImage_1);
        points_2 = detectMinEigenFeatures(grayImage_2); 
    case 'fast'
        points_1 = detectFASTFeatures(grayImage_1);
        points_2 = detectFASTFeatures(grayImage_2);
    case 'harris'
        points_1 = detectHarrisFeatures(grayImage_1);
        points_2 = detectHarrisFeatures(grayImage_2); 
    otherwise
        disp('other value')
end


[features_1, points_1] = extractFeatures(grayImage_1, points_1);
[features_2, points_2] = extractFeatures(grayImage_2, points_2);

% Find correspondences between two image.
[indexPairs, ~] = matchFeatures(features_2, features_1, 'Unique', true);
matchedPoints2 = points_2(indexPairs(:,1), :);
matchedPoints1 = points_1(indexPairs(:,2), :);   



%% Find inlierPoints
estimatEssentialMat = false;

if estimatEssentialMat
    rng(0);
    [E, epipolarInliers] = estimateEssentialMatrix(...
    matchedPoints1, matchedPoints2, cameraParams, 'Confidence', 99.99);

    % Find epipolar inliers
    inlierPoints1 = matchedPoints1(epipolarInliers, :);
    inlierPoints2 = matchedPoints2(epipolarInliers, :);

    % Display inlier matches
    if plot_flag
        figure;
        showMatchedFeatures(I1, I2, inlierPoints1, inlierPoints2);
        title('Epipolar Inliers');
    end
else
    inlierPoints1 = matchedPoints1;
    inlierPoints2 = matchedPoints2;
end    

%% Find corresponding 3D world points -> 2D image points 
points_in_screen = s1.K * s1.points_in_camera; % 3xN in screen coordinate system
uv = cat(1, points_in_screen(1,:)./points_in_screen(3,:), points_in_screen(2,:)./points_in_screen(3,:));
uv = uv';
depth = points_in_screen(3,:)';
imagePoints_1 = inlierPoints1.Location;
imagePoints_2 = inlierPoints2.Location;

% find the best depth for each imagePoints_1 in uv
switch method
    case 'knn'
        [idx,distances]=knnsearch(uv, imagePoints_1,'k',1,'distance','euclidean');    
    otherwise
        disp('other value')
end    

% parameter: set the threshold 
cond = distances < threshold;
idx = idx(cond);

% apply the cond to find the left points
depth = depth(idx);
imagePoints_1 = imagePoints_1(cond,:);
imagePoints_2 = imagePoints_2(cond,:);

% select inlierPoints which has the reasonable 3D depth value.
% uv values should use the original value in "points_reference = inlierPoints1.Location;"
uv_homogeous = cat(2, imagePoints_1(:,1).*depth, imagePoints_1(:,2).*depth, depth);

% project the selected inlierPoints back to 3D world
try 
    points_world = inv(s1.K(1:3,1:3)) * uv_homogeous'; 
catch
    disp('err: in project the selected inlierPoints back to 3D word')
end


%% compute the R and T
% Estimate the world camera pose.
% References
% [1] Gao, X.-S., X.-R. Hou, J. Tang, and H.F. Cheng. "Complete Solution Classification for the Perspective-Three-Point Problem."
% IEEE Transactions on Pattern Analysis and Machine Intelligence. Volume 25,Issue 8, pp. 930–943, August 2003.
% [2] Torr, P. H. S., and A. Zisserman. "MLESAC: A New Robust Estimator with Application to Estimating Image Geometry." 
% Computer Vision and Image Understanding. Volume 78, Issue 1, April 2000, pp. 138-156.

try  
    rng(0);
    [worldOrientation_est,worldLocation_est] = estimateWorldCameraPose(...
        imagePoints_2,points_world',cameraParams);
catch
    disp('err in estimateWorldCameraPose')
end

timesList = toc;


%% error analysis

% translation err
[rotationMatrix_0_to_1,translationVector_0_to_1] = cameraPoseToExtrinsics(s1.pose(:,1:3)',s1.pose(:,end));
worldLocation_true_1 = s2.pose(:,end)' * rotationMatrix_0_to_1 + translationVector_0_to_1; % s2.pose(:,end) in 0 coordinate
err_translation = rssq(worldLocation_est - worldLocation_true_1);

% rotation err
R_0_to_2 = s2.pose(:,1:3)';
R_1_to_0 = s1.pose(:,1:3);
R_1_to_2 = R_0_to_2 * R_1_to_0;
R_est_to_1 = worldOrientation_est';
R_est_to_2 = R_1_to_2 * R_est_to_1;
err_angles = rotm2eul(R_est_to_2) / pi * 180;
err_max_angle = max(abs(err_angles));
 
D = rssq(s1.pose(1:3,end) - s2.pose(1:3,end)); % save the distance between the query and reference image
%% visulize the query and reference images and their camera pose
if plot_flag
    figure;imshowpair(I1, I2, 'montage');
    figure;

    % plot the point cloud in the reference camera coordinate
    pcshow(s1.points_in_camera(1:3,:)','VerticalAxis','Y','VerticalAxisDir','down', ...
     'MarkerSize',30);
    grid on
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    hold on

    % plot the reference camera
    plotCamera('Size',1,'Orientation',eye(3),'Location',...
     zeros(1,3), 'Color', 'b');
    hold on

    % plot the estimated pose of query camera
    plotCamera('Size',1,'Orientation',worldOrientation_est,'Location',...
     worldLocation_est, 'Color', 'r');
    hold on

    % plot the ground true pose of the query camera
    plotCamera('Size',1,'Orientation',R_1_to_2,'Location',...
     worldLocation_true_1, 'Color', 'g');
    title('point cloud in cam1 coordinate');

    plot3(0,0,0, 'b-');
    plot3(worldLocation_est(1),worldLocation_est(2),worldLocation_est(3), 'r-');
    plot3(worldLocation_true_1(1),worldLocation_true_1(2),worldLocation_true_1(3), 'g-');
    legend('3D points','cam1','cam2 estimated', 'cam2 true');

    hold off
end





end

