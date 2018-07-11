function [global_average_info,local_average_info,total_info,instantaneous_reward,cumulative_reward,instantaneous_distances_to_target,cumulative_best_distances_to_target]=activeVision(bagselect,max_iterations, closest_point, radius)
global_average_info=zeros(max_iterations,1);
ego_total_uncertainty=zeros(max_iterations,1);

instantaneous_reward=zeros(max_iterations,1);
cumulative_reward=zeros(max_iterations,1);
instantaneous_distances_to_target=zeros(max_iterations,1);
cumulative_best_distances_to_target=zeros(max_iterations,1);
local_average_info=[];
total_info=[];
%for i=1:bagselect.NumMessages
best_distance=1000000000000.0;
best_reward=-100000000000.0;
best_reward_=-norm(closest_point);

for i=1:max_iterations

    msgs = readMessages(bagselect,i);
    ego_data=msgs{1,1};
    
    reward=-sqrt( (ego_data.FixationPoint.Point.X).^2+...
        (ego_data.FixationPoint.Point.Y).^2+...
        (ego_data.FixationPoint.Point.Z).^2);
    
    if reward> best_reward_
       reward=best_reward_; 
    end
    
    distance_to_target=sqrt( (ego_data.FixationPoint.Point.X-closest_point(1)).^2+...
        (ego_data.FixationPoint.Point.Y-closest_point(2)).^2+...
        (ego_data.FixationPoint.Point.Z-closest_point(3)).^2);

    instantaneous_reward(i,1)=reward;
    instantaneous_distances_to_target(i,1)=distance_to_target;
    
    if i==1
        best_distance=distance_to_target;
        best_reward=reward;
    end
    
    if ~isnan(reward)&&~isinf(reward)
        if reward>best_reward
            
            best_reward=reward;
        end
    else
        return
    end
    
    if ~isnan(distance_to_target)&&~isinf(distance_to_target)
        if distance_to_target<best_distance
            best_distance=distance_to_target;
        end
    else
        return
    end
    
    cumulative_reward(i,1)=best_reward;
    cumulative_best_distances_to_target(i,1)=best_distance;
    
    %     ego_rgb_point_cloud=ego_data.RgbPointCloud;
    %     fieldnames = readAllFieldNames(ego_rgb_point_cloud);
    %     x = readField(ego_rgb_point_cloud,'x');
    %     y = readField(ego_rgb_point_cloud,'y');
    %     z = readField(ego_rgb_point_cloud,'z');
    %     rgb = readRGB(ego_rgb_point_cloud);
    
    ego_point_cloud_uncertainty=ego_data.EgoPointClouds.UncertaintyPointCloud;
    %fieldnames = readAllFieldNames(ego_point_cloud_uncertainty);
    x = readField(ego_point_cloud_uncertainty,'x');
    y = readField(ego_point_cloud_uncertainty,'y');
    z = readField(ego_point_cloud_uncertainty,'z');
    uncertainty = readField(ego_point_cloud_uncertainty,'intensity');
    uncertainty=log(uncertainty);
    uncertainty=uncertainty(~isinf(uncertainty));
    x=x(~isinf(uncertainty));
    y=y(~isinf(uncertainty));
    z=z(~isinf(uncertainty));
    
    total_info=[total_info sum(uncertainty)];
    if sum(isinf(uncertainty))
        uncertainty
    end
    
    ego_points=[x y z];
    
    %% GLOBAL AVERAGE INFORMATION
    global_average_info(i,1)=sum(uncertainty)/20000;
    
    %% CLOSEST POINT AVERAGE INFORMATION (NEIGHBOURHOOD)
    [neighbour_points_idx, d]=rangesearch(ego_points,closest_point, radius);
    
    
    if length(uncertainty(neighbour_points_idx{1}))==0
        local_average_info(i,1)=0;
    else
        local_average_info(i,1)=mean(uncertainty(neighbour_points_idx{1}));
    end
    
    
    
end
end