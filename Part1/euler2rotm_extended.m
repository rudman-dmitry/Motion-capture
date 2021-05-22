function [R]=euler2rotm_extended(euler_angle, euler_seq, rot_type)
% euler_angle = 3 angles
% euler_seq = 'xyz or 'yzx' or 'XYZ' or ...
% rot_type = 'int' or 'ext'

euler_seq = lower(euler_seq);
R = eye(3);
for k = 1 : length(euler_seq) % 3 char for 3 axes
    switch(euler_seq(k))
        case 'x'
            rot_fun=@rotx;
        case 'y'
            rot_fun=@roty;
        case 'z'
            rot_fun=@rotz;
        case '0'
            rot_fun=@(x) eye(3);
    end
    if strcmp(rot_type,'int')
        R = R * rot_fun( euler_angle(k)); %intrinsic
    elseif strcmp(rot_type,'ext') 
        R = rot_fun( euler_angle(k)) * R; %extrinsic
    else
        disp(' rot_type can be ''int'' or ''ext''')
    end
end

end