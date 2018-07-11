%folderpath = fullfile('~/catkin_ws','src');
%rosgenmsg(folderpath)
%addpath('/home/rui/catkin_ws/src/matlab_gen/msggen')
%savepath

%closest_object=[0.384057259734 0.345912799416 0.264831571586];
%closest_object=[0.384057259734 0.418912799416 0.267831571586];
closest_object=[0.331511 0.383341 0.245458];

radius=0.1;

iterations=50;
number_bags=20;
alphas=7;
domain=1:iterations;

alphas_string={'$0$','$0.01$','$1$','$100$','$\infty$','$pi$','$ei$'};
fovs=[90 135];

global_average_info_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
local_average_info_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
total_info_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
instantaneous_distances_to_target_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
cumulative_best_distances_to_target_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
instantaneous_reward_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
cumulative_best_reward_cartesian=zeros(length(fovs),alphas,number_bags,iterations);

global_average_info_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
local_average_info_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
total_info_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
instantaneous_distances_to_target_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
cumulative_best_distances_to_target_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
instantaneous_reward_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
cumulative_best_reward_logpolar=zeros(length(fovs),alphas,number_bags,iterations);

%% PARSE THE DATA
S=sprintf('Parsing data...');
disp(S);
for fov_index=1:length(fovs)
    cartesian_dirs={strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/cartesian/sigma_scale_0.00/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/cartesian/sigma_scale_0.01/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/cartesian/sigma_scale_1.00/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/cartesian/sigma_scale_100.00/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/cartesian/sigma_scale_infinity/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/cartesian/pi/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/cartesian/ei/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/')};
    
    foveal_dirs={strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_0.00/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_0.01/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_1.00/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_100.00/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/sigma_scale_infinity/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/pi/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/'),...        
        strcat('/media/rui/0981-ED8D/rosbags/fov',num2str(fovs(fov_index)),'/200by200/logpolar/ei/mean_0.00_0.00_0.00_std_dev_0.50_0.50_0.50/')};
    
    S=sprintf(' fov: %s ', num2str(fovs(fov_index)));
    disp(S);
    % for each sigma
    for sigma_index=1:length(foveal_dirs)
        S=sprintf(' sigma: %s ', alphas_string{sigma_index});
        disp(S);
        cartesian_dir=cartesian_dirs{sigma_index};
        foveal_dir=foveal_dirs{sigma_index};
        cartesian_files = dir(cartesian_dir);
        foveal_files = dir(foveal_dir);
        
        %% Get data for each file
        for i=1:number_bags
            S=sprintf('   file: %d   ', i);
            disp(S);
            
            %cartesian
            S=sprintf('     cartesian: %s', cartesian_files(i+2).name);
            disp(S);
            
            filePath = strcat(cartesian_dir,cartesian_files(i+2).name);
            bagselect = rosbag(filePath);
            [global_average_info_cartesian(fov_index,sigma_index,i,:),local_average_info_cartesian(fov_index,sigma_index,i,:),total_info_cartesian(fov_index,sigma_index,i,:),instantaneous_reward_cartesian(fov_index,sigma_index,i,:),cumulative_best_reward_cartesian(fov_index,sigma_index,i,:),instantaneous_distances_to_target_cartesian(fov_index,sigma_index,i,:),cumulative_best_distances_to_target_cartesian(fov_index,sigma_index,i,:)]=activeVisionNoPlots(bagselect,iterations,closest_object,radius);
            
            %logpolar
            S=sprintf('     logpolar: %s', foveal_files(i+2).name);
            disp(S);
            filePath = strcat(foveal_dir,foveal_files(i+2).name);
            bagselect = rosbag(filePath);
            plot_space=[-1 5 -2 2 -0.1 2];
            [global_average_info_logpolar(fov_index,sigma_index,i,:),local_average_info_logpolar(fov_index,sigma_index,i,:),total_info_logpolar(fov_index,sigma_index,i,:),instantaneous_reward_logpolar(fov_index,sigma_index,i,:),cumulative_best_reward_logpolar(fov_index,sigma_index,i,:),instantaneous_distances_to_target_logpolar(fov_index,sigma_index,i,:),cumulative_best_distances_to_target_logpolar(fov_index,sigma_index,i,:)]=activeVisionNoPlots(bagselect,iterations,closest_object,radius);
        end
    end
end






