%folderpath = fullfile('~/catkin_ws','src');
%rosgenmsg(folderpath)
%addpath('/home/rui/catkin_ws/src/matlab_gen/msggen')
%savepath

%closest_object=[0.2856 -0.4048 0.0131];
%closest_object=[0.3187 -0.3649 0.3280];
%closest_object=[-1.5-(-1.042069) 0-(0.468433) 0.1636];
clear all
close all
closest_object=[0.384057259734 0.418912799416 0.267831571586];

%closest_object=[0.2811 -0.4021 0.0143];

radius=0.1;

iterations=100;
number_bags=20;
biases=5;
domain=1:iterations;
bias_string={'uniform','top','down','ideal scattered','ideal concentrated'};

fovs=[90];

global_average_info_logpolar=zeros(length(fovs),biases,number_bags,iterations);
local_average_info_logpolar=zeros(length(fovs),biases,number_bags,iterations);
total_info_logpolar=zeros(length(fovs),biases,number_bags,iterations);
instantaneous_distances_to_target_logpolar=zeros(length(fovs),biases,number_bags,iterations);
cumulative_best_distances_to_target_logpolar=zeros(length(fovs),biases,number_bags,iterations);
instantaneous_reward_logpolar=zeros(length(fovs),biases,number_bags,iterations);
cumulative_best_reward_logpolar=zeros(length(fovs),biases,number_bags,iterations);

%% PARSE THE DATA
S=sprintf('Parsing data...');
disp(S);
for fov_index=1:length(fovs)
    
    foveal_dirs={strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_100/mean_0.00_0.00_0.00_std_dev_1.00_1.00_1.00/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_100/mean_0.00_-1.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_100/mean_0.00_1.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_100/mean_-0.67_0.43_0.61_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_100/mean_-0.67_0.43_0.61_std_dev_0.05_0.05_0.05/')};
    
    
    S=sprintf(' fov: %s ', num2str(fovs(fov_index)));
    disp(S);
    % for each sigma
    for bias_index=1:length(foveal_dirs)
        S=sprintf(' bias: %s ', bias_string{bias_index});
        disp(S);
        foveal_dir=foveal_dirs{bias_index};
        foveal_files = dir(foveal_dir);
        
        % for each sigma
        for i=1:number_bags
            S=sprintf('   file: %d   ', i);
            disp(S);
            
            %logpolar
            S=sprintf('     logpolar: %s', foveal_files(i+2).name);
            disp(S);
            filePath = strcat(foveal_dir,foveal_files(i+2).name);
            bagselect = rosbag(filePath);
            plot_space=[-1 5 -2 2 -0.1 2];
            [global_average_info_logpolar(fov_index,bias_index,i,:),local_average_info_logpolar(fov_index,bias_index,i,:),total_info_logpolar(fov_index,bias_index,i,:),instantaneous_reward_logpolar(fov_index,bias_index,i,:),cumulative_best_reward_logpolar(fov_index,bias_index,i,:),instantaneous_distances_to_target_logpolar(fov_index,bias_index,i,:),cumulative_best_distances_to_target_logpolar(fov_index,bias_index,i,:)]=activeVisionNoPlots(bagselect,iterations,closest_object,radius);
        end
    end
end

