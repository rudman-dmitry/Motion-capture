function [f,tau]=kinetic_func(r_cm,r_end,a,R_dis2prox,R_lab2seg,omega,alpha,m,I,GRF,GRM,ver_dir)
% r_cm - ceneter of mass position in lab coordinates - 3D x segments
% r_cm = [thigh,shank,foot]
% r_end - vector from the distal end to ceneter of mass in lab coordinates - 3D x segments
% r_end = [thigh,shank,foot]
% a - ceneter of mass linear acceleration in lab coordinates - 3D x segments
% a = [thigh,shank,foot]
% R_dis2prox - the rotation matrix of the distal segment w.r.t proximal segment - 3 x 3 x segments
% R_dis2prox = [floor2pelvis,thigh2pelvis,shank2thigh,foot2shank]
% R_lab2seg - the rotation matrix of the lab coordinates w.r.t proximal segment - 3 x 3 x segments
% R_lab2seg = [lab2pelvis,lab2thigh,lab2shank,lab2foot]
% omega - the angular velocity of the distal segment w.r.t the proximal segment in distal coordinates - 3D x segments
% omega = [thigh,shank,foot]
% alpha - the angular acceleration of the distal segment w.r.t the proximal segment in distal coordinates - 3D x segments
% alpha = [thigh,shank,foot]
% m - the mass of each segment.
% m = [thigh,shank,foot]
% I - the moment of inertia of the main axes w.r.t the center of mass at the  - 3D x segments
% I = [thigh,shank,foot]
% GRF - Ground Reaction Forces in the lab coordination 3Dx1(minus)
% GRM - Ground Reaction Moments in the lab coordination 3Dx1(minus)
% ver_dir - the vertical direction, i.e. the opposite direction of gravity

g = 9.81* 10^3 *(-ver_dir); %[mm/sec^2]
N_seg = length(m); %number of segments
%inital conditions:
%forces:
f_lab = zeros(3,N_seg+1);
f = zeros(3,N_seg+1);
f_lab(:,end) = -GRF; %[thigh,shank,foot,lab(floor)]
f(:,end) = squeeze(R_lab2seg(:,:,end)) * f_lab(:,end);
%rotation matrix distal2proximal
R_dis2prox_new = cat(3,R_dis2prox(:,:,2:end),R_lab2seg(:,:,end));% [thigh2pelvis,shank2thigh,foot2shank,lab2foot]
%moments:
tau = zeros(3,N_seg+1);
tau(:,end) = -GRM;
for i = N_seg : -1 : 1
    R_l2s = squeeze(R_lab2seg(:,:,i));
    R_d2p = squeeze(R_dis2prox_new(:,:,i));
    %forces
    f_lab(:,i) = f_lab(:,i+1) + m(i)*a(:,i)-m(i)*g;
    f(:,i) = R_l2s * f_lab(:,i);
    %moments:
    tau(:,i) = R_d2p*tau(:,i+1) - cross(f(:,i),R_l2s*r_cm(:,i))...
        +cross(R_d2p*f(:,i+1),R_l2s*r_end(:,i))...
        +diag(I(:,i))*alpha(:,i)+cross(omega(:,i),diag(I(:,i))*omega(:,i));
end
f = f(:,1:end-1);
tau = tau(:,1:end-1);


