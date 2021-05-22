function [pos_rot,coo_sys_rot] = trans_vec(trans,pos,seg_vec,seg_len,coo_vec,coor_len)
% trans = R x segment;
% pos = 3 x segment
% seg_len = 1 x segment
% seg_vec = 3 x segment
% coor_len = 1 x 3d
% coo_vec = 3 x 3d


% calculate the joint position and the coordination system of each joints
N_seg = length(seg_len);
R_pre = eye(3);
coo_sys_rot = zeros(3,3,N_seg);
for k = 1 : N_seg
    R_new = squeeze(trans(:,:,k));
    R_pre = R_pre * R_new;
    vec_rel = R_pre * seg_vec(:,k);
    pos(:,k+1) = pos(:,k)+ seg_len(k) * vec_rel;
    % draw the system coordination
    coo_sys_rot(:,:,k)=repmat(pos(:,k),1,3)+coor_len * R_pre * coo_vec;
end
pos_rot = pos;
end