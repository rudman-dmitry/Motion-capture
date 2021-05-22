function data_out = vicon2rot(data,vicon_seq, data_type_out)
% vicon_seq = 'XZY' intrinsic rotations about axes x-z'-y'' for instance, according to vicon datasheet
% Vicon relative angles - in deg, [-180,180], the order need to be checked
% at the manual, but in general - flexion, abduction, rotation
% convert the relative angles to quat/axang/euler:
% X = flexion
% Z = abduction
% Y =rotation
% all coordinate systems are parrallel when subject stands in N-pose, palms
% forward and thumb to the wright. Y = up, Y=backward, X=right

%check if data type is cell:
if ~iscell(data_type_out)
    data_type_out = {data_type_out};
end    

switch (data_type_out{1})
    case 'euler'
        out_fun = @(x)rotm2eul(x,data_type_out{2});
        data_out = zeros(size(data,1),3);
    case 'axang'
        out_fun = @rotm2axang;
        data_out = zeros(size(data,1),4);
    case 'quat'
        out_fun = @rotm2quat;
        data_out = zeros(size(data,1),4);
    case 'rotm'
        out_fun = @(x) reshape(x,9,1).'; %[col1.', col2.', col3.']
        data_out = zeros(size(data,1),9); %rotation matrix 3X3 into a vec
end

for t = 1 : size(data,1)
    sample = data(t,:);
    if (sum(isnan(sample))>0) %if nan return nan
        out_temp = out_fun(eye(3));
        data_out(t,:) = nan(size(out_temp)); 
    else
        R = eye(3);
        for k = 1 : length(vicon_seq) % 3 char for 3 axes
            axis_ang = vicon_seq(k);
            switch(axis_ang)
                case {'X','x'}
                    rot_fun=@rotx;
                case {'Y','y'}
                    rot_fun=@roty;
                case {'Z','z'}
                    rot_fun=@rotz;
                case '0'
                    rot_fun=@(x) eye(3);
            end    
            R = R * rot_fun( sample(k)); %intrinsic
            %R = rot_fun( sample(k)) * R; %extrinsic
        end
        data_out(t,:) = out_fun(R);
    end
end

end