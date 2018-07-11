%folderpath = fullfile('~/catkin_ws','src');
%rosgenmsg(folderpath)
% addpath('/home/rui/catkin_ws/src/matlab_gen/msggen')
% savepath

%stereo_data = rossubscriber('/stereo',@stereoCallback)
%global data
%global PointStep

close all
rosshutdown
%foveated_stereo_msg = rosmessage('foveated_stereo_ros/Stereo')
rosinit
    
%stereo_sub = rossubscriber('/point_clouds');                                                                                                                                        
ego_sub = rossubscriber('/ego_point_clouds');
plots_figure=figure('units','normalized','outerposition',[0 0 1 1])
set(gcf, 'Color', [1,1,1]);
rgdb_plot_handle=subplot(2,3,1);
uncertainty_plot_handle=subplot(2,3,2);

information_plot_handle=subplot(2,3,3);
ego_information_plot_handle=subplot(2,3,6);

ego_rgdb_plot_handle=subplot(2,3,4);
ego_uncertainty_plot_handle=subplot(2,3,5);
%tightfig;
plot_scale=0.15;
total_uncertainty=[];
ego_total_uncertainty=[];
i=0;
while 1
    i=i+1;
    ego_data=receive(ego_sub,1000);
    stereo_data=ego_data.SensorPointClouds;
    rgb_point_cloud=stereo_data.RgbPointCloud;
    fieldnames = readAllFieldNames(rgb_point_cloud);
    x = readField(rgb_point_cloud,'x');
    y = readField(rgb_point_cloud,'y');
    z = readField(rgb_point_cloud,'z');
    rgb = readRGB(rgb_point_cloud);
    
    subplot(rgdb_plot_handle)
    showPointCloud([y,-x,z],rgb);
    title('RGB-D');
    axis equal
    axis([-1 5 -1 1 -0.1 2])
    
    az = 0.0;
    el = 0;
    view(az, el);
    campos([-325.1602 -262.3470  195.2125]*plot_scale)
    camva(3)
    

    point_cloud_uncertainty=stereo_data.UncertaintyPointCloud;
    fieldnames = readAllFieldNames(point_cloud_uncertainty);
    x = readField(point_cloud_uncertainty,'x');
    y = readField(point_cloud_uncertainty,'y');
    z = readField(point_cloud_uncertainty,'z');
    uncertainty = readField(point_cloud_uncertainty,'intensity');
    uncertainty=log(uncertainty);
    subplot(uncertainty_plot_handle)
    showPointCloud([y,-x,z],(uncertainty));
    title('Uncertainty');
    axis equal
    axis([-1 5 -1 1 -0.1 2])
    
    az = 0.0;
    el = 0;
    view(az, el);
    campos([-325.1602 -262.3470  195.2125]*plot_scale)
    camva(3)
    
    total_uncertainty=[total_uncertainty sum(uncertainty)];
    subplot(information_plot_handle)
    semilogy(total_uncertainty);
    
    string=sprintf('total information %f:', sum(uncertainty));
    disp(string);
    string=sprintf('max information %f:', max(uncertainty));
    disp(string);
    string=sprintf('total points %f:', length(uncertainty));
    disp(string);
    %% EGO SPHERE
    ego_data=ego_data.EgoPointClouds;
    ego_rgb_point_cloud=ego_data.RgbPointCloud;
    fieldnames = readAllFieldNames(rgb_point_cloud);
    x = readField(ego_rgb_point_cloud,'x');
    y = readField(ego_rgb_point_cloud,'y');
    z = readField(ego_rgb_point_cloud,'z');
    rgb = readRGB(ego_rgb_point_cloud);
    
    subplot(ego_rgdb_plot_handle)
    showPointCloud([y,-x,z],rgb);
    title('Ego-Sphere RGB-D');
    axis equal
    axis([-1 5 -1 1 -0.1 2])
    
    az = 0.0;
    el = 0;
    view(az, el);
    campos([-325.1602 -262.3470  195.2125]*plot_scale)
    camva(3)
    
    
    ego_point_cloud_uncertainty=ego_data.UncertaintyPointCloud;
    fieldnames = readAllFieldNames(ego_point_cloud_uncertainty);
    x = readField(ego_point_cloud_uncertainty,'x');
    y = readField(ego_point_cloud_uncertainty,'y');
    z = readField(ego_point_cloud_uncertainty,'z');
    uncertainty = readField(ego_point_cloud_uncertainty,'intensity');
    uncertainty=log(uncertainty);
    subplot(ego_uncertainty_plot_handle)
    showPointCloud([y,-x,z],(uncertainty));
    title('Ego-Sphere Uncertainty');
    axis equal
    axis([-1 5 -1 1 -0.1 2])
    
    az = 0.0;
    el = 0;
    view(az, el);
    campos([-325.1602 -262.3470  195.2125]*plot_scale)
    camva(3)
    
    
    ego_total_uncertainty=[ego_total_uncertainty sum(uncertainty)];
    subplot(ego_information_plot_handle)
    semilogy(ego_total_uncertainty);
    
    string=sprintf('ego total information %f:', sum(uncertainty));
    disp(string);
    string=sprintf('ego max information %f:', max(uncertainty));
    disp(string);
    string=sprintf('ego total points %f:', length(uncertainty));
    disp(string);

%     filename = sprintf('f%03d',i);

    %print(plots_figure,string,'-dpng')  
    %export_fig('-png','-r50',string)
%     disp('saving...');
%     print('-dpng','-r100',filename)
%     disp('done');
%     filename='teste.gif';
%     frame = getframe(1);
%     im = frame2im(frame);
%     [A,map] = rgb2ind(im,256); 
% 	if i == 1;
% 		imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.5);
% 	else
% 		imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.5);
%     end
%pause(1);
end
