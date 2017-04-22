function [ pose ] = read_pose( pose_file, linenum )
%READ_POSE Summary of this function goes here
%   Detailed explanation goes here
    fid = fopen(pose_file);
    data = textscan(fid, '%f', 12, 'delimiter', '\n', 'headerlines', linenum-1);
    pose = reshape(data{1}, [4,3])';
    
    fclose(fid);

end

