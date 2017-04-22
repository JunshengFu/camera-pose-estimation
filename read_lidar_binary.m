function [ output ] = read_lidar_binary( file_path )
%READ_LIDAR_BINARY Summary of this function goes here
%   return the 3xN matrix, each row is a lidar points in velodyne
%   coordinate system

    fid = fopen(file_path);
    data = fread(fid, 'float');
    N = size(data,1)/4;
    data = reshape(data, [4, N]);
    output = data(1:3,:);
    
    fclose(fid);
end

