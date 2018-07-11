%folderpath = fullfile('~/catkin_ws','src');
%rosgenmsg(folderpath)
%addpath('/home/vizzy/catkin_ws/src/matlab_gen/msggen')
%savepath
 
%closest_object=[0.2856 -0.4048 0.0131];
%closest_object=[0.3187 -0.3649 0.3280];
closest_object=[0.3727 0.3410 0.1636];
%closest_object=[0.2811 -0.4021 0.0143];

radius=0.3;
plots_figure=figure('units','normalized','outerposition',[0 0 1 1]);
set(gcf, 'Color', [1,1,1]);
rgdb_plot_handle=subplot(2,3,1);
uncertainty_plot_handle=subplot(2,3,2);
%
information_plot_handle=subplot(2,3,3);
ego_information_plot_handle=subplot(2,3,6);
%
ego_rgdb_plot_handle=subplot(2,3,4);
ego_uncertainty_plot_handle=subplot(2,3,5);

%tightfig;
plot_scale=0.15;
iterations=25;
number_bags=1;

global_average_info_cartesian=zeros(number_bags,iterations);
local_average_info_cartesian=zeros(number_bags,iterations);
total_info_cartesian=zeros(number_bags,iterations);
distances_cartesian=zeros(number_bags,iterations);

legend_string={'$\sigma=0$'};

bag_folder = {'~/rosbags/'};

close all
figure(1);
mean_global_average_info_handle=subplot(1,3,1);
mean_local_average_info_handle=subplot(1,3,2);
mean_distances_handle=subplot(1,3,3);
ColOrd = get(gca,'ColorOrder');
[m,n] = size(ColOrd);
for sigma_index=1:length(bag_folder)
    %% Get data
    for i=1:number_bags
        %cartesian
        filePath = strcat(bag_folder{sigma_index},num2str(i),'.bag');
        bagselect = rosbag(filePath);
        plot_space=[-1 5 -2 2 -0.1 2];
        [global_average_info_cartesian(i,:),local_average_info_cartesian(i,:),total_info_cartesian(i,:),distances_cartesian(i,:)]=activeVisionNoPlots(bagselect,rgdb_plot_handle,uncertainty_plot_handle,information_plot_handle,ego_rgdb_plot_handle,ego_information_plot_handle,ego_uncertainty_plot_handle,plot_space,plot_scale,iterations,closest_object,radius);
    end
    
    %% plot data
    mean_distances_cartesian=mean(distances_cartesian,1);
    mean_global_average_info_cartesian=mean(global_average_info_cartesian,1);
    mean_local_average_info_cartesian=mean(local_average_info_cartesian,1);
    
    std_distances_cartesian=std(distances_cartesian);
    std_global_average_info_cartesian=std(global_average_info_cartesian);
    std_local_average_info_cartesian=std(local_average_info_cartesian);
    
    
    %figure(1)
    %ciplot(-std_dev_distance_cartesian,std_dev_distance_cartesian,mean_distance_cartesian,'b')
    ColRow = rem(sigma_index,m);
    
     % Get the color
    Col = ColOrd(ColRow,:);
    %figure(1)
    subplot(mean_global_average_info_handle)
    hold on
    plot(mean_global_average_info_cartesian','Color',Col,'LineStyle','--')
    L1=legend(legend_string);

    
    subplot(mean_local_average_info_handle)
    hold on
    plot(mean_local_average_info_cartesian','Color',Col,'LineStyle','--')

    
    subplot(mean_distances_handle)
    hold on
    plot(mean_distances_cartesian','Color',Col,'LineStyle','--')
        
    disp('| Sensor |  L/S  |  G/S  |  D/S  |');
    S=sprintf('| cartes | %0.2f | %0.2f | %0.2f  |', double(mean(mean_local_average_info_cartesian)), double(mean(mean_global_average_info_cartesian)), double(mean(mean_distances_cartesian)));
    disp(S);

end
set(gca,'FontSize',15)
set(gcf, 'Color', [1,1,1]);
set(gcf,'defaultaxesposition',[0 0 1 1])
set(L1,'interpreter','latex'); %set Latex interpreter
set(gca,'FontSize',15);
