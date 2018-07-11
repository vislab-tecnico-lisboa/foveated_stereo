close all
cc=lines(12);
load('100by100_conventional_system_with_crop.mat');
figure_handle1=figure('units','normalized','outerposition',[0 0 1 1]);
figure_handle2=figure('units','normalized','outerposition',[0 0 1 1]);
fontsize=25;
figure(figure_handle1)
number_pixels=10000;
average_information=log(average_information)/number_pixels;
%subplot(1,2,1)
%varying distance
vergence_indices=[21 31 41 51];
distance_indices=[21 31 41 51];

a = 1;
b = [1/4 1/4 1/4 1/4];
for i=1:length(vergence_indices)
    plot(distances,average_information(:,vergence_indices(i)),'--','color',cc(i,:),'DisplayName',sprintf('vergence: %0.1f',vergences(vergence_indices(i))*180/pi))
    hold on
end
% 
% L1=legend('-DynamicLegend');
% ylabel('total information','FontSize', fontsize); xlabel('distance (m)','FontSize', fontsize);
% axis([distances(1) 1,min(min(average_information(:,vergence_indices))), max(max(average_information(:,vergence_indices)))])
% set(gca,'FontSize',fontsize)
% set(gcf, 'Color', [1,1,1]);
% set(gcf,'defaultaxesposition',[0 0 1 1])
% set(L1,'interpreter','latex'); %set Latex interpreter
% set(L1,'FontSize',fontsize);
%subplot(1,2,2)
%varying vergence
figure(figure_handle2)
for i=1:length(distance_indices)
    plot(vergences*180/pi,filter(b,a,average_information(distance_indices(i),:)),'--','color',cc(i,:),'DisplayName',sprintf('distance: %0.1f',distances(distance_indices(i))))
    hold on
end

% L2=legend('-DynamicLegend');
% ylabel('total information','FontSize', fontsize); xlabel('vergence (deg)','FontSize', fontsize);
% axis([vergences(1)*180/pi , vergences(end)*180/pi,min(min(average_information(distance_indices,:))), max(max(average_information(distance_indices,:)))])
% set(gca,'FontSize',fontsize)
% set(gcf, 'Color', [1,1,1]);
% set(gcf,'defaultaxesposition',[0 0 1 1])
% set(L2,'interpreter','latex'); %set Latex interpreter
% set(L2,'FontSize',fontsize);



load('100by100_foveated_system_with_crop.mat');
figure(figure_handle1)
%subplot(1,2,1)
%varying distance
average_information=log(average_information)/number_pixels;

for i=1:length(vergence_indices)
    plot(distances,filter(b,a,average_information(:,vergence_indices(i))),'color',cc(i,:),'DisplayName',sprintf('vergence: %0.1f',vergences(vergence_indices(i))*180/pi))
    hold on
end

L3=legend('-DynamicLegend');
ylabel('average information','FontSize', fontsize); xlabel('distance (m)','FontSize', fontsize);
axis([distances(1) 1,min(min(average_information(:,vergence_indices))), max(max(average_information(:,vergence_indices)))])
set(gca,'FontSize',fontsize)
set(gcf, 'Color', [1,1,1]);
set(gcf,'defaultaxesposition',[0 0 1 1])
set(L3,'interpreter','latex'); %set Latex interpreter
set(L3,'FontSize',fontsize);

figure(figure_handle2)
%subplot(1,2,2)
%varying vergence

for i=1:length(distance_indices)
    plot(vergences*180/pi,filter(b,a,average_information(distance_indices(i),:)),'color',cc(i,:),'DisplayName',sprintf('distance: %0.1f',distances(distance_indices(i))))
    hold on
end

L4=legend('-DynamicLegend');
ylabel('average information','FontSize', fontsize); xlabel('vergence (deg)','FontSize', fontsize);
axis([vergences(1)*180/pi, vergences(end)*180/pi,min(min(average_information(distance_indices,:))), max(max(average_information(distance_indices,:)))])
set(gca,'FontSize',fontsize)
set(gcf, 'Color', [1,1,1]);
set(gcf,'defaultaxesposition',[0 0 1 1])
set(L4,'interpreter','latex'); %set Latex interpreter
set(L4,'FontSize',fontsize);

figure(figure_handle1)
string='conv_vs_logpolar_distance';
export_fig('-pdf','-r600',string)


figure(figure_handle2)
string='conv_vs_logpolar_vergence';
export_fig('-pdf','-r600',string)
