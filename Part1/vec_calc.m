function [pos] = calc_vec(seg_len,seg_vec,p0)
% seg_len - length of each segment - 1 x segment
% seg_vec - the direction of each segment - 3 x segment
% p0 - the inital position of the first "joint"
% Calc vec
pos = zeros(3,length(seg_len));
for k = 1 : length(seg_len)
    pos(:,k+1) = pos(:,k) + seg_len(k) * seg_vec(:,k);
end
pos = pos + p0;
end