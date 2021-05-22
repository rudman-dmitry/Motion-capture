function kin = calc_kinematic(data,data_type,Fs,win_wid,euler_seq)
% if euler angles  - must be in rad for comparison with quat


switch data_type
    case 'euler' 
        %for euler angle:
        % theta = the relative angle:
        kin.theta_rot = data;
        %omega = R_dt * R.':
        R_vec = vicon2rot(data,euler_seq, 'rotm');
        R_vec_dt = numeric_diff(R_vec,Fs,'window',win_wid,1); %rotated2fix
        R_vec_d2t = numeric_diff(R_vec,Fs,'window',win_wid,2);
        R_vec_d3t = numeric_diff(R_vec,Fs,'window',win_wid,3);
        
        R_mat = reshape(R_vec.',3,3,[]);
        R_mat_dt = reshape(R_vec_dt.',3,3,[]);
        R_mat_d2t = reshape(R_vec_d2t.',3,3,[]);
        R_mat_d3t = reshape(R_vec_d3t.',3,3,[]);

        N1 = size(R_mat_dt,3);
        N2 = size(R_mat_d2t,3);
        N3 = size(R_mat_d3t,3);
        for k = 1 : N1
            R_temp = squeeze(R_mat(:,:,k));
            R_dt_temp = squeeze(R_mat_dt(:,:,k));
            %omega = R_dt * R.':
            omega_mat = R_dt_temp * transpose(R_temp);
            omega_fix = [omega_mat(3,2) ; omega_mat(1,3) ; omega_mat(2,1)];
            kin.omega_fix(k,:) = (omega_fix).';
            kin.omega_rot(k,:) = (R_temp.' * omega_fix).';
            if k<=N2
                R_d2t_temp = squeeze(R_mat_d2t(:,:,k));
                %alpha = R_d2t * R.' - omega_mat^2:
                alpha_mat = R_d2t_temp * R_temp.' - omega_mat^2;
                alpha_fix = [alpha_mat(3,2) ; alpha_mat(1,3) ; alpha_mat(2,1)];
                kin.alpha_fix(k,:) = (alpha_fix).';
                kin.alpha_rot(k,:) = (R_temp.' * alpha_fix).';
            end
            if k<=N3
                R_d3t_temp = squeeze(R_mat_d3t(:,:,k));
                % jerk = R_d3t * R.' - 2*alpha_mat * omega_mat - omega_mat * R_d2t * R.'
                jerk_mat = R_d3t_temp * R_temp.' - 2 * alpha_mat * omega_mat - omega_mat * R_d2t_temp * R_temp.';
                jerk_fix = [jerk_mat(3,2) ; jerk_mat(1,3) ; jerk_mat(2,1)];
                kin.jerk_fix(k,:) = (jerk_fix).';
                kin.jerk_rot(k,:) = (R_temp.' * jerk_fix).';
            end
        end

    case 'quat' 
        % for quaternion
        % theta = the relative angle:
        kin.theta_rot = eulerd(quaternion(data),euler_seq,'frame');
        %dq = 0.5*omega*q --> omega = 2*dq*conj(q) 
        q = data;
        dq = numeric_diff(data,Fs,'window',win_wid,1);
        d2q = numeric_diff(data,Fs,'window',win_wid,2);
        d3q = numeric_diff(data,Fs,'window',win_wid,3);
        N1 = length(dq);
        omega_q = 2 * quat_mat_times(dq,quat_conj(q(1:N1,:)));
        kin.omega_fix = omega_q(:,2:4);        
        kin.omega_rot = quat_rotation(quat_conj(q(1:N1,:)),kin.omega_fix);       
        %d2q = 0.5(alpha*q+omega*dq) --> alpha =(2*d2q-omega*dq)*conj(q)
        N2 = length(d2q);
        alpha_q = quat_mat_times(2*d2q - quat_mat_times(omega_q(1:N2,:),dq(1:N2,:)),quat_conj(q(1:N2,:)));
        kin.alpha_fix = alpha_q (:,2:4);
        kin.alpha_rot = quat_rotation(quat_conj(q(1:N2,:)),kin.alpha_fix);
        %d3q = 0.5(jerk*q+2*alpha*dq+omega*d2q)->jerk=(2*d3q-2*alpha*dq-omega*d2q)*conj(q)
        N3 = length(d3q);
        jerk_q = quat_mat_times(2*d3q-...
            2*quat_mat_times(alpha_q(1:N3,:),dq(1:N3,:))-quat_mat_times(omega_q(1:N3,:),d2q(1:N3,:)),quat_conj(q(1:N3,:)));
        kin.jerk_fix = jerk_q(:,2:4);
        kin.jerk_rot = quat_rotation(quat_conj(q(1:N3,:)),kin.jerk_fix);
    otherwise
        disp('data type: ''euler'' or ''quat''')
end