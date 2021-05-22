function [data_out] = process_nan(data_in,N)
% linearly intrepulate nan data

nan_ind = isnan(data_in); 
not_nan_ind = not(nan_ind(:,1)); % if one angle is nan all of them are nan
% check if there are nan at the end points of the data:
new_start=[];
new_end=[];
N_temp = N;
if find(not_nan_ind,1)~=1 %nan at the start
    new_start = data_in(find(not_nan_ind,1),:);
    not_nan_ind = [1;not_nan_ind];
    N_temp = N_temp + 1;
end
if find(not_nan_ind,1,'last')~=N_temp %nan at the start
    new_end = data_in(find(not_nan_ind,1,'last'),:);
    not_nan_ind = [not_nan_ind;1];
end
data_in = [new_start;data_in;new_end];
not_nan_ind = logical(not_nan_ind);
all_ind = 1 : size(data_in,1);
data_out = interp1(all_ind(not_nan_ind),data_in(not_nan_ind,:),all_ind);
% delete the additonal data:
if not(isempty(new_start)) %nan at the start
    data_out = data_out(2:end,:);
end
if not(isempty(new_end)) %nan at the start
    data_out = data_out(1:end-1,:);
end    

end