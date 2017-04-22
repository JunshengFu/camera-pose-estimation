classdef ref
    % POSE Summary of this class goes here
    % Detailed explanation goes here
    
    % This is design for the KITTI dataset   
    properties
        Tr                % from Lidar to camera
        K                 % 3x4 matrix, intrinsic, from camera to screen [u v 1]
        points_in_camera  % 4xN matrix, cropped [x y z 1]' , 3D points which are visible at the front camera in camera coordinate system, origin is the camera center, x(right),y(down), z(toward to front)., right handed
        rgb               % the [r g b] (3xN)or [Intensity] (1xN) (grayscale image) value of the points in camera coordinate (with the same order) 
        row               % number of pixel in row
        col               % number of pixel in col
        pose              % camera pose matrix (3x4), from current frame to frame_0
        image             % 2D image
    end
    
    methods

        function obj=ref(rootPath, idx, lidar_flag)
            [obj.Tr, obj.K, obj.image, obj.pose, lidar]=obj.readFiles(rootPath, idx);
            obj.row = size(obj.image,1);
            obj.col = size(obj.image,2);
            if lidar_flag
                [obj.points_in_camera, obj.rgb] = obj.color_lidar(lidar, obj.Tr, obj.K, obj.image);          
            end
        end 
        
        function [Tr, K, image, pose, lidar]=readFiles(obj, rootPath, idx)

            % read calibration file to get Tr and K
            calib = dlmread(rootPath.calibPath, ' ', 0, 1);
            Tr = reshape(calib(end,:), [4,3])';
            K = reshape(calib(1,:), [4,3])';
            
            % read image
            image = imread(fullfile(rootPath.imagePath, strcat(num2str(idx,'%6.6d'), '.png')));
            
            % read lidar 
            lidar = fullfile(rootPath.lidarPath, strcat(num2str(idx,'%6.6d'), '.bin'));
            
            % read pose
            linenum = idx + 1; % the file's name starts with 0
            pose= read_pose( rootPath.posePath, linenum );
        end
        
        function [points_in_camera, rgb_out] = color_lidar(obj, lidarFile, Tr, K, rgb)
            % Input:
            % lidar: lidar file path
            % Tr: 4x4 transform from velodyne to camera coordinate
            % K: 3x4 transform from camera coordinate to screen coordinate
            % rgb: the rgb/intensity value of the image
            % Output:
            % points_in_camera, only contains the points projected in the
            % image frame
            % rgb_out, corresponding rgb info
            
            lidar = read_lidar_binary(lidarFile); % lidar: 3xN lidar points in velodyne coordinates
            if size(Tr,1) == 3
                Tr = cat(1,Tr, [0 0 0 1]); % make Tr homogeneous matrix 4x4
            end
               
            N = size(lidar,2);
            points_in_camera = Tr * cat(1,lidar, ones(1,N));   % 4xN in camera coordinate system
            
            [ ~, points_in_camera, rgb_out] = world2Image( points_in_camera, K, eye(3), zeros(3,1), obj.col, obj.row, rgb );
        end
            
    end
end
