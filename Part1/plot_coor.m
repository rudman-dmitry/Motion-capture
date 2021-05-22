function []=plot_coor(fig_num,pos,coor)
% pos = 3 x segment
for k = 1 : size(coor,3)
    figure(fig_num)
    hold on
    str = {'r','g','b'};
    for c = 1 : 3
        plot3([pos(1,k),squeeze(coor(1,c,k))],...
            [pos(2,k),squeeze(coor(2,c,k))],...
            [pos(3,k),squeeze(coor(3,c,k))],str{c},'linewidth',3)
    end
end
end