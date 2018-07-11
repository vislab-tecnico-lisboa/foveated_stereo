 %% pre process
clear all
close all
load('/media/rui/0981-ED8D/rosbags/data_20_bags_biases1.mat');
bias_string={'uniform','top','down','Target','Target'};
fontsize=15;
chosen_bias=[1 2 3 5];
considered_iterations=50;
instantaneous_reward_logpolar=instantaneous_reward_logpolar(:,chosen_bias,:,1:considered_iterations);
cumulative_best_reward_logpolar=cumulative_best_reward_logpolar(:,chosen_bias,:,1:considered_iterations);
local_average_info_logpolar=local_average_info_logpolar(:,chosen_bias,:,1:considered_iterations);
global_average_info_logpolar=global_average_info_logpolar(:,chosen_bias,:,1:considered_iterations);

reward_data={1}; % instantaneous rewards
regret_data={1}; % instantaneous regret
gap_data={1}; % gap metric
cumulative_best_reward_data={1}; % best reward
distance_to_target_data={1};
best_reward_so_far_data={1};
local_info_data={1};
global_info_data={1};
time_to_best={1};
best_reward=-norm(closest_object);
gap_data_logpolar=zeros(size(instantaneous_reward_logpolar));

instantaneous_reward_logpolar(instantaneous_reward_logpolar>best_reward)=best_reward;
cumulative_best_reward_logpolar(cumulative_best_reward_logpolar>best_reward)=best_reward;

instantaneous_regret_logpolar=best_reward-instantaneous_reward_logpolar;
cumulative_regret_logpolar=cumsum(instantaneous_regret_logpolar,4);

for fov_index=1:length(fovs)
    % Concatenate the data sets from each std_dev in a data x 2 matrix
    reward_data{fov_index}=mean(reshape(instantaneous_reward_logpolar(fov_index,:,:),size(instantaneous_reward_logpolar(fov_index,:,:),2),size(instantaneous_reward_logpolar(fov_index,:,:),3),size(instantaneous_reward_logpolar(fov_index,:,:),4)),3)';
        
    regret_data{fov_index}=mean(reshape(instantaneous_regret_logpolar(fov_index,:,:),size(instantaneous_regret_logpolar(fov_index,:,:),2),size(instantaneous_regret_logpolar(fov_index,:,:),3),size(instantaneous_regret_logpolar(fov_index,:,:),4)),3)';
    
    [test, aux]=max(reshape(instantaneous_reward_logpolar(fov_index,:,:),size(instantaneous_reward_logpolar(fov_index,:,:),2),size(instantaneous_reward_logpolar(fov_index,:,:),3),size(instantaneous_reward_logpolar(fov_index,:,:),4)),[],3);
    time_to_best{fov_index}=aux';
        
    cumulative_best_reward_data{fov_index}=mean(reshape(cumulative_best_reward_logpolar(fov_index,:,:,:),size(cumulative_best_reward_logpolar(fov_index,:,:,:),2),size(cumulative_best_reward_logpolar(fov_index,:,:,:),3),size(cumulative_best_reward_logpolar(fov_index,:,:,:),4)),3)';
    
    distance_to_target_data{fov_index}=mean(reshape(instantaneous_distances_to_target_logpolar(fov_index,:,:,:),size(instantaneous_distances_to_target_logpolar(fov_index,:,:,:),2),size(instantaneous_distances_to_target_logpolar(fov_index,:,:,:),3),size(instantaneous_distances_to_target_logpolar(fov_index,:,:,:),4)),3)';
    
    numerator_log=zeros(size(cumulative_best_reward_logpolar));
    denominator_log=zeros(size(cumulative_best_reward_logpolar));
    for i=1:size(cumulative_best_reward_logpolar(fov_index,:,:,:),4)
        numerator_log(fov_index,:,:,i)=cumulative_best_reward_logpolar(fov_index,:,:,i)-cumulative_best_reward_logpolar(fov_index,:,:,1);
        denominator_log(fov_index,:,:,i)=best_reward-cumulative_best_reward_logpolar(fov_index,:,:,1);
    end
    gap_data_logpolar(fov_index,:,:,:)=numerator_log(fov_index,:,:,:)./denominator_log(fov_index,:,:,:);
    
    gap_data{fov_index}=mean(reshape(gap_data_logpolar(fov_index,:,:,:),size(gap_data_logpolar(fov_index,:,:,:),2),size(gap_data_logpolar(fov_index,:,:,:),3),size(gap_data_logpolar(fov_index,:,:,:),4)),3)';
    
    local_info_data{fov_index}=mean(reshape(local_average_info_logpolar(fov_index,:,:,:),size(local_average_info_logpolar(fov_index,:,:,:),2),size(local_average_info_logpolar(fov_index,:,:,:),3),size(local_average_info_logpolar(fov_index,:,:,:),4)),3)';
    
    global_info_data{fov_index}=mean(reshape(global_average_info_logpolar(fov_index,:,:,:),size(global_average_info_logpolar(fov_index,:,:,:),2),size(global_average_info_logpolar(fov_index,:,:,:),3),size(global_average_info_logpolar(fov_index,:,:,:),4)),3)';
end
%%plots
close all
figure(1);
set(gcf, 'Color', [1,1,1]);
% mean_distances_handle_90=subplot(1,3,1);
% title('average reward')
% xlabel('saccade')

mean_regret_handle_90=subplot(1,4,3);
ylabel('cumulative regret','FontSize',fontsize)
xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);

mean_average_gap_handle_90=subplot(1,4,1);
ylabel('gap reduction','FontSize',fontsize)
xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);

mean_local_average_info_handle_90=subplot(1,4,2);
ylabel('average local information','FontSize',fontsize)
xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);

mean_global_average_info_handle_90=subplot(1,4,4);
ylabel('average global information','FontSize',fontsize)
xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);
% mean_distances_handle_135=subplot(2,4,4);
% title('average reward')
% xlabel('saccade')

%mean_reward_handle=[mean_distances_handle_90 mean_distances_handle_135];
regret_handle=[mean_regret_handle_90];
gap_handle=[mean_average_gap_handle_90];
mean_local_average_info_handle=[mean_local_average_info_handle_90];
mean_global_average_info_handle=[mean_global_average_info_handle_90];

%ColOrd = get(gca,'ColorOrder');
%[m,n] = size(ColOrd);
h1s=[];
h2s=[];
h3s=[];
h4s=[];
alpha_plots=[1 2 3 4];
colors=rand(length(foveal_dirs),3);
colors(alpha_plots(1),:)=[1 0 0];
colors(alpha_plots(2),:)=[0 1 0];
colors(alpha_plots(3),:)=[0 0 1];
colors(alpha_plots(4),:)=[1 0.5490 0];



for fov_index=1:length(fovs)
    for i=1:length(alpha_plots)
        sigma_index=alpha_plots(i);
        figure(fov_index);
        %plot rewards
%         subplot(mean_reward_handle(fov_index))
%         hold on
%         
%         y=reshape(instantaneous_reward_logpolar(fov_index,sigma_index,:,:),size(instantaneous_reward_logpolar,3),size(instantaneous_reward_logpolar,4));
%         h1=plot(mean(y)','Color',colors(sigma_index,:));
%         h1s=[h1s;h1];
%         
%         y=reshape(instantaneous_reward_cartesian(fov_index,sigma_index,:,:),size(instantaneous_reward_cartesian,3),size(instantaneous_reward_cartesian,4));
%         plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
%         
        %plot regrets
        subplot(regret_handle(fov_index))
        hold on
        
        y=reshape(cumulative_regret_logpolar(fov_index,sigma_index,:,:),size(cumulative_regret_logpolar,3),size(cumulative_regret_logpolar,4));
        h1=plot(mean(y)','Color',colors(sigma_index,:));
        h1s=[h1s;h1];
        xlim([1 50]);
       
        %plot average local information
        subplot(gap_handle(fov_index))
        hold on
        
        y=reshape(gap_data_logpolar(fov_index,sigma_index,:,:),size(gap_data_logpolar,3),size(gap_data_logpolar,4));
        h2=plot(mean(y)','Color',colors(sigma_index,:));
        h2s=[h2s;h2];
        xlim([1 50]);

        %plot average local information
        subplot(mean_local_average_info_handle(fov_index))
        hold on
        
        y=reshape(local_average_info_logpolar(fov_index,sigma_index,:,:),size(local_average_info_logpolar,3),size(local_average_info_logpolar,4));
        h3=plot(mean(y)','Color',colors(sigma_index,:));
        h3s=[h3s;h3];
        xlim([1 50]);
       
        %plot average global information
        subplot(mean_global_average_info_handle(fov_index))
        hold on
        
        y=reshape(global_average_info_logpolar(fov_index,sigma_index,:,:),size(global_average_info_logpolar,3),size(global_average_info_logpolar,4));
        h4=plot(mean(y)','Color',colors(sigma_index,:));
        h4s=[h4s;h4];
        xlim([1 50]);

    end
end


for fov_index=1:length(fovs)
%     subplot(mean_reward_handle(fov_index))
%     legend(h1s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),bias_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
%     
    %subplot(regret_handle(fov_index))
    %legend(h1s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),bias_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
    
    subplot(gap_handle(fov_index))
    legend(h2s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),bias_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
    
    %subplot(mean_local_average_info_handle(fov_index))
    %legend(h3s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),bias_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
    
    %subplot(mean_global_average_info_handle(fov_index))
    %legend(h4s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),bias_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
    
end
  
%% Box plots
legend_string={'foveal fov-90º','foveal fov-135º','cartesian fov-90º','cartesian fov-135º'};
colors=cell(4,biases);
for i=1:biases
    colors{1,i}=[0 1 1];
    colors{2,i}=[0 0 1];
    colors{3,i}=[1 0.5490 0];
    colors{4,i}=[1 0 0];
end


figure(3)
set(gcf, 'Color', [1,1,1]);
% 
% instantaneous_reward_error_bar_logpolar_handle=subplot(1,4,1);
% title('average reward per saccade')

average_regret_bar_logpolar_handle=subplot(1,4,3);

gap_error_bar_handle=subplot(1,4,1);

local_average_info_error_bar_logpolar_handle=subplot(1,4,2);

global_average_info_error_bar_logpolar_handle=subplot(1,4,4);


% subplot(instantaneous_reward_error_bar_logpolar_handle)
% aboxplot(reward_data,colors,'labels',bias_string); % Advanced box plot
% xlabel('$\alpha$','Interpreter','LaTex'); % Set the X-axis
% plotTickLatex2D();
% legend(legend_string,'Location','southeast'); % Add a legend

subplot(average_regret_bar_logpolar_handle)
aboxplot(regret_data,colors,'labels',bias_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
h=ylabel('average cumulative regret per saccade','FontSize',fontsize);
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);

subplot(gap_error_bar_handle)
aboxplot(gap_data,colors,'labels',bias_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis
plotTickLatex2D('FontSize',fontsize);
h=ylabel('average gap reduction per saccade','FontSize',fontsize);
set(h, 'Units', 'Normalized', 'Position', [-0.24, 0.5, 0]);

subplot(local_average_info_error_bar_logpolar_handle)
aboxplot(local_info_data,colors,'labels',bias_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
h=ylabel('mean average local information per saccade','FontSize',fontsize);
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);

subplot(global_average_info_error_bar_logpolar_handle)
aboxplot(global_info_data,colors,'labels',bias_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
h=ylabel('mean average global information per saccade','FontSize',fontsize);
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);

