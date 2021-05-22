function []=plot_stick_fig(fig_num, pos)
% pos = 3 x segment
% plot
figure(fig_num)
scatter3(pos(1,:),pos(2,:),pos(3,:),'filled')
hold on
plot3(pos(1,:),pos(2,:),pos(3,:),'linewidth',2)
axis equal
xlabel('x (forward)');ylabel('y (left)');zlabel('z (up)');
text(pos(1,:),pos(2,:),pos(3,:),{'O','H','K','A','T'},'FontSize',14)
% view(-47,29)
end