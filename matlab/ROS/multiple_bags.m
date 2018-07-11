close all
set(0,'defaulttextinterpreter','latex')
%folderpath = fullfile('~/catkin_ws','src');
%rosgenmsg(folderpath)
%addpath('/home/rui/catkin_ws/src/matlab_gen/msggen')
%savepath

%closest_object=[0.2856 -0.4048 0.0131];
%closest_object=[0.3187 -0.3649 0.3280];
%closest_object=[-1.5-(-1.042069) 0-(0.468433) 0.1636];

closest_object=[0.384057259734 0.385912799416 0.264831571586];

%closest_object=[0.2811 -0.4021 0.0143];

radius=0.1;
%plots_figure=figure('units','normalized','outerposition',[0 0 1 1]);
%set(gcf, 'Color', [1,1,1]);
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
iterations=100;
number_bags=2;
alphas=6;
domain=1:iterations;

alphas_string={'$0$','$1$','$10$','$100$','$1000$','$\infty$'};
fovs=[90 135];

%for i=1:length(fovs)
%    cartesian_dirs={strcat('~/rosbags/fov',num2str(fovs(1)),'/100by100/cartesian/sigma_scale_0/')};
%end
global_average_info_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
local_average_info_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
total_info_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
distances_cartesian=zeros(length(fovs),alphas,number_bags,iterations);
best_distances_cartesian=zeros(length(fovs),alphas,number_bags,iterations);

global_average_info_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
local_average_info_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
total_info_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
distances_logpolar=zeros(length(fovs),alphas,number_bags,iterations);
best_distances_logpolar=zeros(length(fovs),alphas,number_bags,iterations);

dist_data={4};
local_info_data={4};
global_info_data={4};
%% PARSE THE DATA
S=sprintf('Parsing data...');
disp(S);
for fov_index=1:length(fovs)
    cartesian_dirs={strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/cartesian/sigma_scale_0/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/cartesian/sigma_scale_1/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/cartesian/sigma_scale_10/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/cartesian/sigma_scale_100/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/cartesian/sigma_scale_1000/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/cartesian/sigma_scale_infinity/')};
    
    foveal_dirs={strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/logpolar/sigma_scale_0/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/logpolar/sigma_scale_1/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/logpolar/sigma_scale_10/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/logpolar/sigma_scale_100/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/logpolar/sigma_scale_1000/'),...
        strcat('~/rosbags/fov',num2str(fovs(fov_index)),'/100by100/logpolar/sigma_scale_infinity/')};
    


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
            S=sprintf('   file: %d ', i);
            disp(S);
            
            %cartesian
            S=sprintf('     cartesian');
            disp(S);
            
            filePath = strcat(cartesian_dir,cartesian_files(i+2).name);
            bagselect = rosbag(filePath);
            plot_space=[-1 5 -2 2 -0.1 2];
            [global_average_info_cartesian(fov_index,sigma_index,i,:),local_average_info_cartesian(fov_index,sigma_index,i,:),total_info_cartesian(fov_index,sigma_index,i,:),distances_cartesian(fov_index,sigma_index,i,:)]=activeVisionNoPlots(bagselect,rgdb_plot_handle,uncertainty_plot_handle,information_plot_handle,ego_rgdb_plot_handle,ego_information_plot_handle,ego_uncertainty_plot_handle,plot_space,plot_scale,iterations,closest_object,radius);
            
            %logpolar
            S=sprintf('     logpolar');
            disp(S);
            filePath = strcat(foveal_dir,foveal_files(i+2).name);
            bagselect = rosbag(filePath);
            plot_space=[-1 5 -2 2 -0.1 2];
            [global_average_info_logpolar(fov_index,sigma_index,i,:),local_average_info_logpolar(fov_index,sigma_index,i,:),total_info_logpolar(fov_index,sigma_index,i,:),distances_logpolar(fov_index,sigma_index,i,:)]=activeVisionNoPlots(bagselect,rgdb_plot_handle,uncertainty_plot_handle,information_plot_handle,ego_rgdb_plot_handle,ego_information_plot_handle,ego_uncertainty_plot_handle,plot_space,plot_scale,iterations,closest_object,radius);
        end
    end
    
    mean_distances_logpolar= mean(mean(distances_logpolar(:,:,2:end),3),2);
    mean_distances_cartesian=mean(mean(distances_cartesian(:,:,2:end),3),2);
    
    mean_local_average_info_logpolar= mean(mean(local_average_info_logpolar,3),2);
    mean_local_average_info_cartesian=mean(mean(local_average_info_cartesian,3),2);
    
    mean_global_average_info_logpolar= mean(mean(global_average_info_logpolar,3),2);
    mean_global_average_info_cartesian=mean(mean(global_average_info_cartesian,3),2);
    
    
    std_distances_logpolar= std(mean(distances_logpolar(:,:,2:end),3),0,2);
    std_distances_cartesian=std(mean(distances_cartesian(:,:,2:end),3),0,2);
    
    std_local_average_info_logpolar= std(mean(local_average_info_logpolar, 3),0,2);
    std_local_average_info_cartesian=std(mean(local_average_info_cartesian,3),0,2);
    
    std_global_average_info_logpolar= std(mean(global_average_info_logpolar, 3),0,2);
    std_global_average_info_cartesian=std(mean(global_average_info_cartesian,3),0,2);
    
    % Concatenate the data sets from each std_dev in a data x 2 matrix
    dist_data{fov_index}=mean(reshape(distances_logpolar(fov_index,:,:,:),size(distances_logpolar,2),size(distances_logpolar,3),size(distances_logpolar,4)),3)';
    dist_data{2+fov_index}=mean(reshape(distances_cartesian(fov_index,:,:,:),size(distances_cartesian,2),size(distances_cartesian,3),size(distances_cartesian,4)),3)';
    
    local_info_data{fov_index}=mean(reshape(local_average_info_logpolar(fov_index,:,:,:),size(local_average_info_logpolar,2),size(local_average_info_logpolar,3),size(local_average_info_logpolar,4)),3)';
    local_info_data{2+fov_index}=mean(reshape(local_average_info_cartesian(fov_index,:,:,:),size(local_average_info_cartesian,2),size(local_average_info_cartesian,3),size(local_average_info_cartesian,4)),3)';
    
    global_info_data{fov_index}=mean(reshape(global_average_info_logpolar(fov_index,:,:,:),size(global_average_info_logpolar,2),size(global_average_info_logpolar,3),size(global_average_info_logpolar,4)),3)';
    global_info_data{2+fov_index}=mean(reshape(global_average_info_cartesian(fov_index,:,:,:),size(global_average_info_cartesian,2),size(global_average_info_cartesian,3),size(global_average_info_cartesian,4)),3)';
    
end


%%plots

figure(1);
set(gcf, 'Color', [1,1,1]);
mean_distances_handle_90=subplot(2,3,1);
title('average instantaneous reward')
xlabel('saccade')

mean_local_average_info_handle_90=subplot(2,3,2);
title('average local information')
xlabel('saccade')

mean_global_average_info_handle_90=subplot(2,3,3);
title('average global information')
xlabel('saccade')

mean_distances_handle_135=subplot(2,3,4);
title('average instantaneous reward')
xlabel('saccade')

mean_local_average_info_handle_135=subplot(2,3,5);
title('average local information')
xlabel('saccade')

mean_global_average_info_handle_135=subplot(2,3,6);
title('average global information')
xlabel('saccade')


mean_distances_handle=[mean_distances_handle_90 mean_distances_handle_135];
mean_local_average_info_handle=[mean_local_average_info_handle_90 mean_local_average_info_handle_135];
mean_global_average_info_handle=[mean_global_average_info_handle_90 mean_global_average_info_handle_135];

%ColOrd = get(gca,'ColorOrder');
%[m,n] = size(ColOrd);

colors=rand(length(foveal_dirs),3);
for fov_index=1:length(fovs)
    for sigma_index=1:length(foveal_dirs)
        
        %plot distances
        subplot(mean_distances_handle(fov_index))
        hold on
        y=reshape(distances_logpolar(fov_index,sigma_index,:,:),size(distances_logpolar,3),size(distances_logpolar,4));
        plot(mean(y)','Color',colors(sigma_index,:));
        
        y=reshape(distances_cartesian(fov_index,sigma_index,:,:),size(distances_cartesian,3),size(distances_cartesian,4));
        plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
        
        %plot average local information
        subplot(mean_local_average_info_handle(fov_index))
        hold on
        y=reshape(local_average_info_logpolar(fov_index,sigma_index,:,:),size(local_average_info_logpolar,3),size(local_average_info_logpolar,4));
        plot(mean(y)','Color',colors(sigma_index,:));
        
        y=reshape(local_average_info_cartesian(fov_index,sigma_index,:,:),size(local_average_info_logpolar,3),size(local_average_info_logpolar,4));
        plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
        
        
        %plot average global information
        subplot(mean_global_average_info_handle(fov_index))
        hold on
        y=reshape(global_average_info_logpolar(fov_index,sigma_index,:,:),size(global_average_info_logpolar,3),size(global_average_info_logpolar,4));
        plot(mean(y)','Color',colors(sigma_index,:));
        
        y=reshape(global_average_info_cartesian(fov_index,sigma_index,:,:),size(global_average_info_cartesian,3),size(global_average_info_cartesian,4));
        plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
    end
end

%%%%%%%%%%%
%% PLOTS %%
%%%%%%%%%%%

%% TIME EVOLUTION PLOTS

% for sigma_index=1:length(foveal_dirs)
%     
%     % logpolar
%     subplot(mean_global_average_info_handle)
%     hold on
%     y=reshape(global_average_info_logpolar(sigma_index,:,:),size(global_average_info_logpolar,2),size(global_average_info_logpolar,3));
%     ColRow = rem(sigma_index,m);
%     % Get the color
%     Col = ColOrd(ColRow,:);
%     shadedErrorBar(domain, y, {@mean, @(domain) 1*std(domain)}, {'Color',Col}, 1);
%     
%     subplot(mean_local_average_info_handle)
%     hold on
%     y=reshape(local_average_info_logpolar(sigma_index,:,:),size(local_average_info_logpolar,2),size(local_average_info_logpolar,3));
%     ColRow = rem(sigma_index,m);
%     % Get the color
%     Col = ColOrd(ColRow,:);
%     shadedErrorBar(domain, y, {@mean, @(domain) 1*std(domain)  }, {'Color',Col}, 1);
%     
%     subplot(mean_distances_handle)
%     hold on
%     y=reshape(distances_logpolar(sigma_index,:,:),size(distances_logpolar,2),size(distances_logpolar,3));
%     ColRow = rem(sigma_index,m);
%     % Get the color
%     Col = ColOrd(ColRow,:);
%     shadedErrorBar(domain, y, {@mean, @(domain) 1*std(domain)  }, {'Color',Col}, 1);
%     
%     % cartesian
%     subplot(mean_global_average_info_handle)
%     hold on
%     y=reshape(global_average_info_cartesian(sigma_index,:,:),size(global_average_info_cartesian,2),size(global_average_info_cartesian,3));
%     ColRow = rem(sigma_index,m);
%     % Get the color
%     Col = ColOrd(ColRow,:);
%     shadedErrorBar(domain, y, {@mean, @(domain) 1*std(domain)  }, {'Color',Col,'LineStyle','--'}, 1);
%     
%     subplot(mean_local_average_info_handle)
%     hold on
%     y=reshape(local_average_info_cartesian(sigma_index,:,:),size(local_average_info_cartesian,2),size(local_average_info_cartesian,3));
%     ColRow = rem(sigma_index,m);
%     % Get the color
%     Col = ColOrd(ColRow,:);
%     shadedErrorBar(domain, y, {@mean, @(domain) 1*std(domain)  }, {'Color',Col,'LineStyle','--'}, 1);
%     
%     subplot(mean_distances_handle)
%     hold on
%     y=reshape(distances_cartesian(sigma_index,:,:),size(distances_cartesian,2),size(distances_cartesian,3));
%     ColRow = rem(sigma_index,m);
%     % Get the color
%     Col = ColOrd(ColRow,:);
%     shadedErrorBar(domain, y, {@mean, @(domain) 1*std(domain)  }, {'Color',Col,'LineStyle','--'}, 1);
% end

%% Box plots
legend_string={'foveal fov-90º','foveal fov-135º','cartesian fov-90º','cartesian fov-135º'};

figure(3)
set(gcf, 'Color', [1,1,1]);

mean_distances_error_bar_logpolar_handle=subplot(1,3,1);
title('average reward per saccade')

mean_local_average_info_error_bar_logpolar_handle=subplot(1,3,2);
title('average local information per saccade')

mean_global_average_info_error_bar_logpolar_handle=subplot(1,3,3);
title('average global information per saccade')

subplot(mean_distances_error_bar_logpolar_handle)
aboxplot(dist_data,'labels',alphas_string); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex'); % Set the X-axis
plotTickLatex2D();
legend(legend_string,'Location','southeast'); % Add a legend

subplot(mean_local_average_info_error_bar_logpolar_handle)
aboxplot(local_info_data,'labels',alphas_string); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex'); % Set the X-axis label
plotTickLatex2D();
legend(legend_string,'Location','southeast'); % Add a legend

subplot(mean_global_average_info_error_bar_logpolar_handle)
aboxplot(global_info_data,'labels',alphas_string,'Interpreter','LaTex'); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex'); % Set the X-axis label
plotTickLatex2D();
legend(legend_string,'Location','southeast'); % Add a legend



% set(gca,'FontSize',15)
% set(gcf, 'Color', [1,1,1]);
% set(gcf,'defaultaxesposition',[0 0 1 1])
% set(L1,'interpreter','latex'); %set Latex interpreter
% set(gca,'FontSize',15);
% L1=legend(legend_string);



%figure(1)
% %ciplot(-std_dev_distance_cartesian,std_dev_distance_cartesian,mean_distance_cartesian,'b')
% ColRow = rem(sigma_index,m);
%
% % Get the color
% Col = ColOrd(ColRow,:);
% %figure(1)
% subplot(mean_global_average_info_handle)
% hold on
% plot(mean_global_average_info_cartesian','Color',Col,'LineStyle','--')
% L1=legend(legend_string);
% hold on
% plot(mean_global_average_info_logpolar','Color',Col)
% xlim([1 iterations])
%
% subplot(mean_local_average_info_handle)
% hold on
% plot(mean_local_average_info_cartesian','Color',Col,'LineStyle','--')
% hold on
% plot(mean_local_average_info_logpolar','Color',Col)
% xlim([1 iterations])
%
% subplot(mean_distances_handle)
% hold on
% plot(mean_distances_cartesian','Color',Col,'LineStyle','--')
% hold on
% plot(mean_distances_logpolar','Color',Col)
% xlim([1 iterations]);

disp('|   alpha    | Sensor |  D/S |  L/S |  G/S  |');
for sigma_index=1:length(foveal_dirs)
    S=sprintf('| %s | cartes | %0.2f | %0.2f | %0.2f  |', alphas_string{sigma_index}, double(mean_distances_cartesian(sigma_index)), double(mean_local_average_info_cartesian(sigma_index)), double(mean_global_average_info_cartesian(sigma_index)));
    disp(S);
    S=sprintf('| %s | log-po | %0.2f | %0.2f | %0.2f  |', alphas_string{sigma_index}, double(mean_distances_logpolar(sigma_index)),  double(mean_local_average_info_logpolar(sigma_index)), double(mean_global_average_info_logpolar(sigma_index)));
    disp(S);
end




