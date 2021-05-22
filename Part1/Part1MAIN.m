% Part 1
close all; clear; clc;

dataD = load('dynamic4.mat');
dataS = load('static.mat');
N = size(dataD.subject.joint.angles.data,1);
N_analog = size(dataD.analog.emg.data,1);

%% stick figure 

N_joints = length(dataD.subject.joint.angles.names);
N_seg = (N_joints-1)/2 + 1;
N_joint_side = (N_joints-1)/2;
stick_vec_L = [0 1 0; 0 0 -1; 0 0 -1; 1 0 0].';
stick_vec_R = [0 -1 0; 0 0 -1; 0 0 -1; 1 0 0].';
stick_fig_seg_len = [0.3,1,1,0.3]
p0_pelvis = [0 0 2].';
joints_pos_int_L = vec_calc(stick_fig_seg_len,stick_vec_L,p0_pelvis);
joints_pos_int_R = vec_calc(stick_fig_seg_len,stick_vec_R,p0_pelvis);

% coor param:
coor_seg_vec = [1 0 0; 0 1 0; 0 0 1].';%x ,y,z
coor_seg_len = 0.1;%x,y,z
%trans param
rot_type = 'int';
euler_seq = 'yxz';
% angles data:
angle_yxz_all = reshape(dataD.subject.joint.angles.data_yxz,N,3,N_joints); 
%% get rid of nan:
fig_num = 1;
yxz_all_p_ang = zeros(size(angle_yxz_all));
h(fig_num)=figure(fig_num);
for j = 1:N_joints
    angles_temp = squeeze(angle_yxz_all(:,:,j));
    angles_new = get_rid_of_nan(angles_temp,N);
    yxz_all_p_ang(:,:,j) = angles_new;
    %plot:
    subplot(2,4,j)
    plot(angles_temp)
    hold on
    plot(angles_new,'--')
    title(dataD.subject.joint.angles.names{j})
end
legend({'x','y','z','xn','yn','zn'},'position',[0.7928,0.3049,0.0360,0.1230])
fig_num = fig_num + 1;
%% plot stick figure:
IsMovie = 0;
if IsMovie
for t = 1 : N
    euler_angle_L = squeeze(yxz_all_p_ang(t,:,[1,1+(1:N_joint_side)]));
    euler_angle_R = squeeze(yxz_all_p_ang(t,:,[1,1+(N_joint_side+1:N_joints-1)]));
    trans_L = repmat(eye(3),1,1,N_seg);
    trans_R = repmat(eye(3),1,1,N_seg);
    for k = 1 : N_joint_side+1
        trans_L(:,:,k) = euler2rotm_extended(euler_angle_L(:,k), euler_seq, rot_type);
        trans_R(:,:,k) = euler2rotm_extended(euler_angle_R(:,k), euler_seq, rot_type);
    end
    
    [joints_pos_rot_L,coor_pos_rot_L] = trans_vec(trans_L,joints_pos_int_L,stick_vec_L,stick_fig_seg_len,coor_seg_vec,coor_seg_len);
    [joints_pos_rot_R,coor_pos_rot_R] = trans_vec(trans_R,joints_pos_int_R,stick_vec_R,stick_fig_seg_len,coor_seg_vec,coor_seg_len);

    plot_stick_fig(fig_num, joints_pos_rot_L)
    plot_stick_fig(fig_num, joints_pos_rot_R)
    plot_coor(fig_num,joints_pos_rot_L,coor_pos_rot_L);
    plot_coor(fig_num,joints_pos_rot_R,coor_pos_rot_R);
    view(-15,29)
    xlim([-1.5,1.5]);ylim([-1,1]);zlim([0,2.1]);
    frames(t) = getframe(gcf);
    hold off
end
% create movie:
writerObj = VideoWriter('walk.avi');
writerObj.FrameRate = dataD.subject.Fs;
open(writerObj);
for t = 1 : N
    writeVideo(writerObj,frames(t));
end
close(writerObj);
fig_num = fig_num + 1;
% play movie
%implay(['outputs\',file_name,'.avi'])
end
%% Find to gait cycle:
t_vec = (0 : N-1)/dataD.subject.Fs;
ankle_name_ind = find(contains(dataD.subject.marker.names,{'LANK','RANK'}));
ankle_ind = 3*ankle_name_ind; %z is the third element
ankle_z_L = get_rid_of_nan(dataD.subject.marker.data(:,ankle_ind(1)),N);
ankle_z_R = get_rid_of_nan(dataD.subject.marker.data(:,ankle_ind(2)),N);
h(fig_num)=figure(fig_num);
plot(t_vec,ankle_z_L,t_vec,ankle_z_R)
legend('Left ankle marker','Right Ankle marker')
% find the cycle gait start according to the minimal z value of the ankle:
[~,Llocs] = findpeaks(-ankle_z_L,'MinPeakProminence',4,'MinPeakDistance',50);
[~,Rlocs] = findpeaks(-ankle_z_R,'MinPeakProminence',3,'MinPeakDistance',50);
%Rlocs = Rlocs(1:end-1);
hold on
scatter(t_vec(Llocs),ankle_z_L(Llocs),50,'b','filled')
scatter(t_vec(Rlocs),ankle_z_R(Rlocs),50,'r','filled')
legend('LANK','RANK','Left side starting point','Right side starting point')
xlabel('t [sec]'); 
ylabel('Z coordinate of the marker [mm]')
hold off
fig_num = fig_num + 1;
%% Biomechanical angles:
angle_bio_all = reshape(dataD.subject.joint.angles.data,N,3,N_joints);
angle_bio_all_p = zeros(size(angle_bio_all));
for j = 1:N_joints
    angles_temp = squeeze(angle_bio_all(:,:,j));
    angle_bio_all_p(:,:,j) = get_rid_of_nan(angles_temp,N);
end
%% Split gait cycles:
euler_angle_L_p = angle_bio_all_p(:,:,2:4);
euler_angle_R_p = angle_bio_all_p(:,:,5:7);
p = 0 : 99;
[angles_gait_L,angles_gait_L_avg] = gc_split(euler_angle_L_p,Llocs,t_vec, p);
[angles_gait_R,angles_gait_R_avg] = gc_split(euler_angle_R_p,Rlocs,t_vec, p);
%plot
h(fig_num)=figure(fig_num);
str_angle = {'flexion','adduction','rotation'};
for k = 1 : size(angles_gait_R_avg,3)
    for a = 1 : 3
        subplot(3,3,3*(k-1)+a)
        plot(p,angles_gait_L_avg(:,a,k))
        hold on
        plot(p,angles_gait_R_avg(:,a,k))
        title([dataD.subject.joint.angles.names{k+1}(2:end),' - ',str_angle{a}])
        xlabel('% Gait cycle');
        ylabel('Joint angle[deg]');
        
    end
end
legend('Left', 'Right', 'orientation','horizontal','position',[0.4547,0.0259,0.1414,0.0233])
fig_num = fig_num + 1;

%% forces:
GRF1 = dataD.analog.fp.data(:,1:3);
GRM1 = dataD.analog.fp.data(:,4:6);
GRF2 = dataD.analog.fp.data(:,7:9);
GRM2 = dataD.analog.fp.data(:,10:12);
t_ana = (0:N_analog-1)/dataD.analog.Fs;
%plot
dir_str = {'X - direction','Y - direction','Z - direction'};
h(fig_num)=figure(fig_num);
for k = 1 : 3
    subplot(3,2,2*k-1)
    plot(t_ana,GRF1(:,k), 'LineWidth', 1.5)
    hold on
    plot(t_ana,GRF2(:,k), 'LineWidth', 1.5)
    title(['Force in - ',dir_str{k}])
    xlabel('time (sec)');ylabel('f[N]');
    axis tight
    grid on
    subplot(3,2,2*k)
    plot(t_ana,GRM1(:,k), 'LineWidth', 1.5)
    hold on
    plot(t_ana,GRM2(:,k), 'LineWidth', 1.5)
    title(['moment in - ',dir_str{k}])
    xlabel('time (sec)');
    ylabel(' Moment [N*mm]');
    legend('Right','Left', 'orientation','horizontal','position',[0.4547,0.0259,0.1414,0.0233])
    axis tight
    grid on
end
fig_num = fig_num+1;

%% Downsampling

ind_FP1 =  find(abs(GRF1(:,1))>eps,1):find(abs(GRF1(:,1))>eps,1,'last');
ind_FP2 =  find(abs(GRF2(:,1))>eps,1):find(abs(GRF2(:,1))>eps,1,'last');
t_start = t_ana(ind_FP1(1)); t_stop = t_ana(ind_FP1(end));
GRF_R = downsample(GRF1(ind_FP1,:),dataD.analog.Fs/dataD.subject.Fs);
GRM_R = downsample(GRM1(ind_FP1,:),dataD.analog.Fs/dataD.subject.Fs);
RGRFt = find(and(t_vec>=t_start,t_vec<=t_stop));
if numel(GRF_R)>numel(RGRFt)
    RGRFt = [RGRFt(1)-1,RGRFt];
end

figure(10)

for k = 1 : 3
    subplot(3,1,k)
    plot(RGRFt/120, GRF_R(:,k), 'LineWidth', 1.5)
    title(['Force in - ',dir_str{k}])
    xlabel('t[sec]');
    hold on
end

%% arrange joints centers:
JC_names = {'RHJC','RKJC','RAJC'};
joint_est_pos = zeros(N,3,length(JC_names));
for j = 1 : length(JC_names)
    ind_JC = find(contains(dataD.subject.joint.center.names,JC_names{j}));
    data_temp = dataD.subject.joint.center.data(:,3*ind_JC-2:3*ind_JC);
    joint_est_pos(:,:,j) = get_rid_of_nan(data_temp,N);
end
toe_ind = find(contains(dataD.subject.marker.names,'RTOE'));
toe_data = dataD.subject.marker.data(:,3*toe_ind-2:3*toe_ind);
joint_est_pos = cat(3,joint_est_pos,get_rid_of_nan(toe_data,N)); %hip,knee,ankle,toe
%% calc velocities and acceleration:
athro_param_prox = [0.433;0.433;0.5];
win_wid = 4;
euler_seq = 'yxz';
l_seg = zeros(1,N_joint_side);
r_cm = zeros(N,3,N_joint_side);
a_cm = zeros(N-win_wid*2,3,N_joint_side);
omega = zeros(N-win_wid*1,3,N_joint_side);alpha = zeros(N-win_wid*2,3,N_joint_side);
%linear:
for j = 1:N_joint_side %hip,knee,ankle
    l_seg(:,j) = norm(squeeze(mean(joint_est_pos(:,:,j)-joint_est_pos(:,:,j+1))));
    r_cm_temp = squeeze(joint_est_pos(:,:,j))*(1-athro_param_prox(j))+ ...
        squeeze(joint_est_pos(:,:,j+1))*athro_param_prox(j);
    a_cm_temp = numeric_diff(r_cm_temp,dataD.subject.Fs,'window',win_wid,2);
    r_cm(:,:,j) = r_cm_temp;
    a_cm(:,:,j) = a_cm_temp;
end
%angular:
angle_yxz_all_p_R = yxz_all_p_ang(:,:,[1,5:7]);
for j = 1:N_joint_side+1 
    angles = squeeze(angle_yxz_all_p_R(:,:,j));
    kin = calc_kinematic(angles,'euler',dataD.subject.Fs,win_wid,euler_seq);
    omega(:,:,j) = kin.omega_rot;
    alpha(:,:,j) = kin.alpha_rot;
end
% plot
h(fig_num)=figure(fig_num);
str_seg = {'Thigh','Shank','Foot'};
str_kin = {'r_{cm} [mm]','a_{cm} [mm/sec^2]','\omega [rad/sec]','\alpha [rad/sec^2]'};
kin_plot{1} = r_cm;kin_plot{2} = a_cm;kin_plot{3} = omega(:,:,2:end);kin_plot{4} = alpha(:,:,2:end);
for k = 1 :3
    for m = 1 : length(kin_plot)
        subplot(3,4,length(kin_plot)*(k-1)+m)
        data_temp = squeeze(kin_plot{m}(:,:,k));
        N_temp = size(data_temp,1);
        plot(t_vec(end-N_temp+1:end),data_temp)
        title([str_seg{k},'-',str_kin{m}])
        xlabel('t [sec]');
        axis tight
        grid on
    end
end
legend({'x','y','z'},'orientation','horizontal','position',[0.4653,0.0381,0.0928,0.0233])    
fig_num = fig_num+1;
%% Forces

R_dis2prox = zeros(3,3,N,N_joint_side+1);
R_lab2seg = zeros(3,3,N,N_joint_side+1);
for t = 1 : N
    R=eye(3);
    for j = 1:N_joint_side+1
        ang_temp = angle_yxz_all_p_R(t,:,j);
        R_dis2prox_temp = euler2rotm_extended(ang_temp, euler_seq, 'int');
        R_dis2prox(:,:,t,j) = R_dis2prox_temp;
        R = R * R_dis2prox_temp;
        R_lab2seg(:,:,t,j) = transpose(R);
    end
end

r_cm_i = zeros(N,3,N_joint_side);
r_end2cm_i = zeros(N,3,N_joint_side);
for t = 1 : N
    R=eye(3);
    for j = 1:N_joint_side  
        R_l2s_temp = squeeze(R_lab2seg(:,:,t,j+1));
        v_temp = (squeeze(joint_est_pos(t,:,j+1))-squeeze(joint_est_pos(t,:,j))).';
        r_cm_i(t,:,j) = R_l2s_temp *(athro_param_prox(j)*v_temp);
        r_end2cm_i(t,:,j) = R_l2s_temp *((1-athro_param_prox(j))*(-v_temp));
    end
end

g=9.81;
M=max(mean(dataS.analog.fp.data(:,[3,9])))/g;
ver_dir = [0;0;1];
m = [0.1*M;0.0465*M; 0.0145*M];
I_main = m .* ([0.323;0.302;0.475].*l_seg.').^2;
I = [I_main(1),I_main(1),I_main(1)/5; I_main(2),I_main(2),I_main(2)/5; I_main(3)/5 ,I_main(3),I_main(3)];
f = zeros(3,N_joint_side,length(RGRFt));
tau = zeros(3,N_joint_side,length(RGRFt));
for ind = 1 : length(RGRFt)
    r_temp = squeeze(r_cm_i(RGRFt(ind),:,:));
    r_end_temp = squeeze(r_end2cm_i(RGRFt(ind),:,:));
    a_temp = squeeze(a_cm(RGRFt(ind),:,:));
    omega_temp = squeeze(omega(RGRFt(ind),:,:));
    alpha_temp = squeeze(alpha(RGRFt(ind),:,:));
    R_d2p = squeeze(R_dis2prox(:,:,RGRFt(ind),:));
    R_l2s = squeeze(R_lab2seg(:,:,RGRFt(ind),:));
    GRF = 10^3 * GRF_R(ind,:).'; 
    GRM = 10^3 * GRM_R(ind,:).'; 
    [f(:,:,ind),tau(:,:,ind)]=kinetic_func(r_temp,r_end_temp,a_temp,R_d2p,R_l2s,omega_temp,alpha_temp,m,I,GRF,GRM,ver_dir);
end
f = f *10^-3; 
tau = tau * 10^-3;
%% force plot
h(fig_num)=figure(fig_num);
str_joints = {'Hip','Knee','Ankle'};
for k = 1 : N_joint_side
    if k ==N_joint_side 
        f_temp = squeeze(f(:,k,[3,2,1]));
        tau_temp = squeeze(tau(:,k,[3,2,1]));
    else %
        f_temp = squeeze(f(:,k,:));
        tau_temp = squeeze(tau(:,k,:));
    end
    subplot(3,2,2*k-1)
    plot(t_vec(RGRFt),squeeze(f(:,k,:)), 'LineWidth', 1.5)
    title(['force - ',str_joints{k}])
    xlabel('time [sec]');ylabel('force[N]');
    legend({'frontal','lateral','longitudinal'},'location','westoutside')
    axis tight
    grid on
    subplot(3,2,2*k)
    plot(t_vec(RGRFt),squeeze(tau(:,k,:)), 'LineWidth', 1.5)
    title(['moment - ',str_joints{k}])
    xlabel('time [sec]');ylabel('moment[N*mm]');
    legend({'abduction','flexion','rotation'},'location','eastoutside')
    axis tight
    grid on
end

fig_num=fig_num+1;
