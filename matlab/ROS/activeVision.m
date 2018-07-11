function [total_uncertainty,ego_total_uncertainty, fixation_points]=activeVision(bagselect,rgdb_plot_handle,uncertainty_plot_handle,information_plot_handle,ego_rgdb_plot_handle,ego_information_plot_handle,ego_uncertainty_plot_handle,plot_space,plot_scale,max_iterations, closest_point)
total_uncertainty=[];
ego_total_uncertainty=[];
fixation_points=[];
for i=1:max_iterations
    msgs = readMessages(bagselect,i);
    
    ego_data=msgs{1,1};
    
    fixation_point=ego_data.FixationPoint.Point;
    fixation_points=[fixation_points; fixation_point.X fixation_point.Y fixation_point.Z];
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
    axis(plot_space)
    hold on
    scatter3(fixation_point.Y,-fixation_point.X,fixation_point.Z,30,'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75]);
    hold off
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
    hold on
    scatter3(fixation_point.Y,-fixation_point.X,fixation_point.Z,30,'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75]);
    hold off
    title('Information');
    axis equal
    axis(plot_space)
    
    az = 0.0;
    el = 0;
    view(az, el);
    campos([-325.1602 -262.3470  195.2125]*plot_scale)
    camva(3)
    
    total_uncertainty=[total_uncertainty sum(uncertainty)];
    subplot(information_plot_handle)
    semilogy(total_uncertainty);
    title('total information');
    axis([1 bagselect.NumMessages 10^4 10^6])
    
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
    axis(plot_space)
    hold on
    scatter3(fixation_point.Y,-fixation_point.X,fixation_point.Z,30,'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75]);
    hold off
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
    title('Ego-Sphere Information');
    axis equal
    axis(plot_space)
    hold on
    scatter3(fixation_point.Y,-fixation_point.X,fixation_point.Z,30,'MarkerEdgeColor','k',...
        'MarkerFaceColor',[0 .75 .75]);
    hold off
    az = 0.0;
    el = 0;
    view(az, el);
    campos([-325.1602 -262.3470  195.2125]*plot_scale)
    camva(3)
    
    
    ego_total_uncertainty=[ego_total_uncertainty sum(uncertainty)];
    subplot(ego_information_plot_handle)
    semilogy(ego_total_uncertainty);
    axis([1 bagselect.NumMessages 10^4 5*10^5])
    
    title('Ego total information');
    
    
    
    string=sprintf('ego total information %f:', sum(uncertainty));
    disp(string);
    string=sprintf('ego max information %f:', max(uncertainty));
    disp(string);
    string=sprintf('ego total points %f:', length(uncertainty));
    disp(string);
    drawnow
    
    filename='teste.gif';
    frame = getframe(1);
    im = frame2im(frame);
    [A,map] = rgb2ind(im,256);
    if i == 1;
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',1.0);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',1.0);
    end
    %
end
end