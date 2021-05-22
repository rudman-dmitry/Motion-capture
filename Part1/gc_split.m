function [data_out,data_avg] = split_gait_cycle(data_in,locs,t,p)
% split data into gait cycle and average
[~,D,J] = size(data_in);

data_out = zeros(length(p),D,J,length(locs)-1);
for j = 1 : J
    data_temp = squeeze(data_in(:,:,j));
    for k = 1 : length(locs)-1
        t_temp = linspace(t(locs(k)), t(locs(k+1)-1),length(p));
        data_out(:,:,j,k) = interp1(t(locs(k):locs(k+1)-1),data_temp(locs(k):locs(k+1)-1,:),t_temp);
    end
end
data_avg = mean(data_out,4);
end