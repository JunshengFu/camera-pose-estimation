function [ points_in_screen, points_in_camera, rgb_out] = world2Image( points_in_camera, K, R, T, col, row, rgb )
%WORLD2IMAGE project 3d world point to 2d image plane
%   Detailed explanation goes here
% Inputs
% points_in_camera, 3xN
% K, intrinsic
% R, Rotation Matrix from world to image
% T, Rotation Vector from world to image
% col, number of cols in image
% row, number of rows in image
% rgb, image intensity or rgb values

% Outputs
% points_in_screen, 2
% points_in_camera, 
% rgb_out

    if size(points_in_camera, 1) == 3
        N = size(points_in_camera, 2);
        points_in_camera = cat(1, points_in_camera, ones(1, N));
    end

    points_in_camera_ = cat(2, R, T) * points_in_camera;
    points_in_screen = K * points_in_camera;            % 3xN in screen coordinate system
    [~,column] = find(points_in_screen(3,:)<=0);           % remove the point cloud at the back of the image plane
    points_in_screen(:,column)=[];                         % remove the corresponding points in screen coordinate
    points_in_camera(:,column)=[];                         % remove the corresponding points in camera coordinate                         

    uv = cat(1, points_in_screen(1,:)./points_in_screen(3,:), points_in_screen(2,:)./points_in_screen(3,:));

    % crop the image
    inside = (uv(1,:)            <= col);
    inside = and(inside, uv(1,:) >= 0);
    inside = and(inside, uv(2,:) <= row);
    inside = and(inside, uv(2,:) >= 0);
    uv_inside = uv(:,inside);

    points_in_camera = points_in_camera(:,inside);

    uv = round(uv_inside(1:2,:)); % only round the uv value, not the depth value

    [~, col1] = find(uv(1:2,:)<1); % handle the image boundaries
    [~, col2] = find(uv(1,:) > col | uv(2,:) > row); % handle the image boundaries
    uv(:,col1) = [];
    uv(:,col2) = [];

    points_in_camera(:,col1) = [];
    points_in_camera(:,col2) = [];

    u = uv(1,:);
    v = uv(2,:);

    linearidx = sub2ind([row, col], v, u);
    if size(rgb,3) == 1      % if it is a grayscale image
        rgb_out = rgb(linearidx); 
    elseif size(rgb,3) == 3  % if it is a rgb image
        r = rgb(:,:,1);
        g = rgb(:,:,2);
        b = rgb(:,:,3);
        r_out = r(linearidx);
        g_out = g(linearidx);
        b_out = b(linearidx);

        rgb_out = cat(1, r_out, g_out, b_out);
    else
        error('it is neither a grayscale image nor rgb image')
    end

end

