%% pre process
clear all
close all

load('/media/rui/0981-ED8D/rosbags/test2.mat');

considered_iterations=40;
chosen_alphas=[1 2 3 4 5 6 7];
alphas_string=alphas_string(chosen_alphas);
alphas_string=alphas_string(1:5)
chosen_alhpa=4;
instantaneous_reward_logpolar=instantaneous_reward_logpolar(:,chosen_alphas,:,1:considered_iterations);
instantaneous_reward_cartesian=instantaneous_reward_cartesian(:,chosen_alphas,:,1:considered_iterations);
cumulative_best_reward_logpolar=cumulative_best_reward_logpolar(:,chosen_alphas,:,1:considered_iterations);
cumulative_best_reward_cartesian=cumulative_best_reward_cartesian(:,chosen_alphas,:,1:considered_iterations);
local_average_info_logpolar=local_average_info_logpolar(:,chosen_alphas,:,1:considered_iterations);
local_average_info_cartesian=local_average_info_cartesian(:,chosen_alphas,:,1:considered_iterations);
global_average_info_logpolar=global_average_info_logpolar(:,chosen_alphas,:,1:considered_iterations);
global_average_info_cartesian=global_average_info_cartesian(:,chosen_alphas,:,1:considered_iterations);

fontsize=15;
reward_data={4}; % instantaneous rewards
regret_data={4}; % instantaneous regret
gap_data={4}; % gap metric
cumulative_best_reward_data={4}; % best reward
distance_to_target_data={4};
best_reward_so_far_data={4};
local_info_data={4};
global_info_data={4};
time_to_best={4};
best_reward=-norm(closest_object);
gap_data_logpolar=zeros(size(instantaneous_reward_logpolar));
gap_data_cartesian=zeros(size(instantaneous_reward_cartesian));

instantaneous_reward_logpolar(instantaneous_reward_logpolar>best_reward)=best_reward;
instantaneous_reward_cartesian(instantaneous_reward_cartesian>best_reward)=best_reward;

cumulative_best_reward_logpolar(cumulative_best_reward_logpolar>best_reward)=best_reward;
cumulative_best_reward_cartesian(cumulative_best_reward_cartesian>best_reward)=best_reward;


instantaneous_regret_logpolar=best_reward-instantaneous_reward_logpolar;
instantaneous_regret_cartesian=best_reward-instantaneous_reward_cartesian;

cumulative_regret_logpolar=cumsum(instantaneous_regret_logpolar,4);
cumulative_regret_cartesian=cumsum(instantaneous_regret_cartesian,4);

for fov_index=1:length(fovs)
    % Concatenate the data sets from each std_dev in a data x 2 matrix
    reward_data{fov_index}=mean(reshape(instantaneous_reward_logpolar(fov_index,:,:),size(instantaneous_reward_logpolar(fov_index,:,:),2),size(instantaneous_reward_logpolar(fov_index,:,:),3),size(instantaneous_reward_logpolar(fov_index,:,:),4)),3)';
    reward_data{2+fov_index}=mean(reshape(instantaneous_reward_cartesian(fov_index,:,:),size(instantaneous_reward_cartesian(fov_index,:,:),2),size(instantaneous_reward_cartesian(fov_index,:,:),3),size(instantaneous_reward_cartesian(fov_index,:,:),4)),3)';
    
    regret_data{fov_index}=mean(reshape(instantaneous_regret_logpolar(fov_index,:,:),size(instantaneous_regret_logpolar(fov_index,:,:),2),size(instantaneous_regret_logpolar(fov_index,:,:),3),size(instantaneous_regret_logpolar(fov_index,:,:),4)),3)';
    regret_data{2+fov_index}=mean(reshape(instantaneous_regret_cartesian(fov_index,:,:),size(instantaneous_regret_cartesian(fov_index,:,:),2),size(instantaneous_regret_cartesian(fov_index,:,:),3),size(instantaneous_regret_cartesian(fov_index,:,:),4)),3)';
    
    [test, aux]=max(reshape(instantaneous_reward_logpolar(fov_index,:,:),size(instantaneous_reward_logpolar(fov_index,:,:),2),size(instantaneous_reward_logpolar(fov_index,:,:),3),size(instantaneous_reward_logpolar(fov_index,:,:),4)),[],3);
    time_to_best{fov_index}=aux';
    
    [test, aux]=max(reshape(instantaneous_reward_cartesian(fov_index,:,:),size(instantaneous_reward_cartesian(fov_index,:,:),2),size(instantaneous_reward_cartesian(fov_index,:,:),3),size(instantaneous_reward_cartesian(fov_index,:,:),4)),[],3);
    time_to_best{2+fov_index}=aux';
    
    cumulative_best_reward_data{fov_index}=mean(reshape(cumulative_best_reward_logpolar(fov_index,:,:,:),size(cumulative_best_reward_logpolar(fov_index,:,:,:),2),size(cumulative_best_reward_logpolar(fov_index,:,:,:),3),size(cumulative_best_reward_logpolar(fov_index,:,:,:),4)),3)';
    cumulative_best_reward_data{2+fov_index}=mean(reshape(cumulative_best_reward_cartesian(fov_index,:,:,:),size(cumulative_best_reward_cartesian(fov_index,:,:,:),2),size(cumulative_best_reward_cartesian(fov_index,:,:,:),3),size(cumulative_best_reward_cartesian(fov_index,:,:,:),4)),3)';
    
    distance_to_target_data{fov_index}=mean(reshape(instantaneous_distances_to_target_logpolar(fov_index,:,:,:),size(instantaneous_distances_to_target_logpolar(fov_index,:,:,:),2),size(instantaneous_distances_to_target_logpolar(fov_index,:,:,:),3),size(instantaneous_distances_to_target_logpolar(fov_index,:,:,:),4)),3)';
    distance_to_target_data{2+fov_index}=mean(reshape(instantaneous_distances_to_target_cartesian(fov_index,:,:,:),size(instantaneous_distances_to_target_cartesian(fov_index,:,:,:),2),size(instantaneous_distances_to_target_cartesian(fov_index,:,:,:),3),size(instantaneous_distances_to_target_cartesian(fov_index,:,:,:),4)),3)';
    
    numerator_log=zeros(size(cumulative_best_reward_logpolar));
    denominator_log=zeros(size(cumulative_best_reward_logpolar));
    for i=1:size(cumulative_best_reward_logpolar(fov_index,:,:,:),4)
        numerator_log(fov_index,:,:,i)=cumulative_best_reward_logpolar(fov_index,:,:,i)-cumulative_best_reward_logpolar(fov_index,:,:,1);
        denominator_log(fov_index,:,:,i)=best_reward-cumulative_best_reward_logpolar(fov_index,:,:,1);
    end
    gap_data_logpolar(fov_index,:,:,:)=numerator_log(fov_index,:,:,:)./denominator_log(fov_index,:,:,:);
    
    numerator_cart=zeros(size(cumulative_best_reward_cartesian));
    denominator_cart=zeros(size(cumulative_best_reward_cartesian));
    for i=1:size(cumulative_best_reward_cartesian(fov_index,:,:,:),4)
        numerator_cart(fov_index,:,:,i)=cumulative_best_reward_cartesian(fov_index,:,:,i)-cumulative_best_reward_cartesian(fov_index,:,:,1);
        denominator_cart(fov_index,:,:,i)=best_reward-cumulative_best_reward_cartesian(fov_index,:,:,1);
    end
    gap_data_cartesian(fov_index,:,:,:)=numerator_cart(fov_index,:,:,:)./denominator_cart(fov_index,:,:,:);
    
    gap_data{fov_index}=mean(reshape(gap_data_logpolar(fov_index,:,:,:),size(gap_data_logpolar(fov_index,:,:,:),2),size(gap_data_logpolar(fov_index,:,:,:),3),size(gap_data_logpolar(fov_index,:,:,:),4)),3)';
    gap_data{fov_index+2}=mean(reshape(gap_data_cartesian(fov_index,:,:,:),size(gap_data_cartesian(fov_index,:,:,:),2),size(gap_data_cartesian(fov_index,:,:,:),3),size(gap_data_cartesian(fov_index,:,:,:),4)),3)';
    
    local_info_data{fov_index}=mean(reshape(local_average_info_logpolar(fov_index,:,:,:),size(local_average_info_logpolar(fov_index,:,:,:),2),size(local_average_info_logpolar(fov_index,:,:,:),3),size(local_average_info_logpolar(fov_index,:,:,:),4)),3)';
    local_info_data{2+fov_index}=mean(reshape(local_average_info_cartesian(fov_index,:,:,:),size(local_average_info_cartesian(fov_index,:,:,:),2),size(local_average_info_cartesian(fov_index,:,:,:),3),size(local_average_info_cartesian(fov_index,:,:,:),4)),3)';
    
    global_info_data{fov_index}=mean(reshape(global_average_info_logpolar(fov_index,:,:,:),size(global_average_info_logpolar(fov_index,:,:,:),2),size(global_average_info_logpolar(fov_index,:,:,:),3),size(global_average_info_logpolar(fov_index,:,:,:),4)),3)';
    global_info_data{2+fov_index}=mean(reshape(global_average_info_cartesian(fov_index,:,:,:),size(global_average_info_cartesian(fov_index,:,:,:),2),size(global_average_info_cartesian(fov_index,:,:,:),3),size(global_average_info_cartesian(fov_index,:,:,:),4)),3)';
end
%%plots
% close all
% figure(1);
% set(gcf, 'Color', [1,1,1]);
% % mean_distances_handle_90=subplot(1,3,1);
% % title('average reward')
% % xlabel('saccade')
%
% mean_regret_handle_90=subplot(1,4,1);
% ylabel('mean cumulative regret','FontSize',fontsize)
% xlabel('saccade','FontSize',fontsize)
% set(gca,'fontsize',fontsize);
%
% mean_average_gap_handle_90=subplot(1,4,2);
% ylabel('average gap reduction','FontSize',fontsize)
% xlabel('saccade','FontSize',fontsize)
% set(gca,'fontsize',fontsize);
%
% mean_local_average_info_handle_90=subplot(1,4,3);
% ylabel('average local information','FontSize',fontsize)
% xlabel('saccade','FontSize',fontsize)
% set(gca,'fontsize',fontsize);
%
% mean_global_average_info_handle_90=subplot(1,4,4);
% ylabel('average global information','FontSize',fontsize)
% xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);
% mean_distances_handle_135=subplot(2,4,4);
% title('average reward')
% xlabel('saccade')
% figure(2);
% set(gcf, 'Color', [1,1,1]);
% mean_regret_handle_135=subplot(1,4,1);
% ylabel('mean cumulative regret','FontSize',fontsize)
% xlabel('saccade','FontSize',fontsize)
% set(gca,'fontsize',fontsize);
%
% mean_local_average_gap_handle_135=subplot(1,4,2);
% ylabel('mean gap reduction','FontSize',fontsize)
% xlabel('saccade','FontSize',fontsize)
% set(gca,'fontsize',fontsize);
%
% mean_local_average_info_handle_135=subplot(1,4,3);
% ylabel('mean average local information','FontSize',fontsize)
% xlabel('saccade','FontSize',fontsize)
% set(gca,'fontsize',fontsize);
%
% mean_global_average_info_handle_135=subplot(1,4,4);
% ylabel('mean average global information','FontSize',fontsize)
% xlabel('saccade','FontSize',fontsize)
% set(gca,'fontsize',fontsize);
%
% %mean_reward_handle=[mean_distances_handle_90 mean_distances_handle_135];
% regret_handle=[mean_regret_handle_90 mean_regret_handle_135];
% gap_handle=[mean_average_gap_handle_90 mean_local_average_gap_handle_135];
% mean_local_average_info_handle=[mean_local_average_info_handle_90 mean_local_average_info_handle_135];
% mean_global_average_info_handle=[mean_global_average_info_handle_90 mean_global_average_info_handle_135];
%
% %ColOrd = get(gca,'ColorOrder');
% %[m,n] = size(ColOrd);
% h1s=[];
% h2s=[];
% h3s=[];
% h4s=[];
% alpha_plots=[1 3 5];
% colors=rand(length(foveal_dirs),3);
% colors(alpha_plots(1),:)=[1 0 0];
% colors(alpha_plots(2),:)=[0 1 0];
% colors(alpha_plots(3),:)=[0 0 1];
%
%
% for fov_index=1:length(fovs)
%     for i=1:length(alpha_plots)
%         sigma_index=alpha_plots(i);
%         figure(fov_index);
%         %plot rewards
% %         subplot(mean_reward_handle(fov_index))
% %         hold on
% %
% %         y=reshape(instantaneous_reward_logpolar(fov_index,sigma_index,:,:),size(instantaneous_reward_logpolar,3),size(instantaneous_reward_logpolar,4));
% %         h1=plot(mean(y)','Color',colors(sigma_index,:));
% %         h1s=[h1s;h1];
% %
% %         y=reshape(instantaneous_reward_cartesian(fov_index,sigma_index,:,:),size(instantaneous_reward_cartesian,3),size(instantaneous_reward_cartesian,4));
% %         plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
% %
%         %plot regrets
%         subplot(regret_handle(fov_index))
%         hold on
%
%         y=reshape(cumulative_regret_logpolar(fov_index,sigma_index,:,:),size(cumulative_regret_logpolar,3),size(cumulative_regret_logpolar,4));
%         h1=plot(mean(y)','Color',colors(sigma_index,:));
%         h1s=[h1s;h1];
%
%         y=reshape(cumulative_regret_cartesian(fov_index,sigma_index,:,:),size(cumulative_regret_cartesian,3),size(cumulative_regret_cartesian,4));
%         plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
%
%
%         %plot average local information
%         subplot(gap_handle(fov_index))
%         hold on
%
%         y=reshape(gap_data_logpolar(fov_index,sigma_index,:,:),size(gap_data_logpolar,3),size(gap_data_logpolar,4));
%         h2=plot(mean(y)','Color',colors(sigma_index,:));
%         h2s=[h2s;h2];
%
%         y=reshape(gap_data_cartesian(fov_index,sigma_index,:,:),size(gap_data_cartesian,3),size(gap_data_cartesian,4));
%         plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
%
%
%         %plot average local information
%         subplot(mean_local_average_info_handle(fov_index))
%         hold on
%
%         y=reshape(local_average_info_logpolar(fov_index,sigma_index,:,:),size(local_average_info_logpolar,3),size(local_average_info_logpolar,4));
%         h3=plot(mean(y)','Color',colors(sigma_index,:));
%         h3s=[h3s;h3];
%
%         y=reshape(local_average_info_cartesian(fov_index,sigma_index,:,:),size(local_average_info_cartesian,3),size(local_average_info_cartesian,4));
%         plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
%
%
%         %plot average global information
%         subplot(mean_global_average_info_handle(fov_index))
%         hold on
%
%         y=reshape(global_average_info_logpolar(fov_index,sigma_index,:,:),size(global_average_info_logpolar,3),size(global_average_info_logpolar,4));
%         h4=plot(mean(y)','Color',colors(sigma_index,:));
%         h4s=[h4s;h4];
%
%         y=reshape(global_average_info_cartesian(fov_index,sigma_index,:,:),size(global_average_info_cartesian,3),size(global_average_info_cartesian,4));
%         plot(mean(y)','Color',colors(sigma_index,:),'LineStyle','--');
%     end
% end
%
%
% for fov_index=1:length(fovs)
% %     subplot(mean_reward_handle(fov_index))
% %     legend(h1s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),alphas_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
% %
%     subplot(regret_handle(fov_index))
%     legend(h1s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),alphas_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
%
%     subplot(gap_handle(fov_index))
%     legend(h2s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),alphas_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
%
%     subplot(mean_local_average_info_handle(fov_index))
%     legend(h3s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),alphas_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
%
%     subplot(mean_global_average_info_handle(fov_index))
%     legend(h4s(((fov_index-1)*length(alpha_plots)+1):(fov_index*length(alpha_plots))),alphas_string(alpha_plots),'Interpreter','LaTex','Location','southeast');
%
% end
%



local_average_info_cartesian(:,:,:,1)=0;



figure(1);
set(gcf, 'Color', [1,1,1]);
gap_handle=subplot(1,4,1);
ylabel('gap reduction','FontSize',fontsize)
xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);

mean_local_average_info_handle=subplot(1,4,2);
ylabel('average local information','FontSize',fontsize)
xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);

regret_handle=subplot(1,4,3);
ylabel('cumulative regret','FontSize',fontsize)
xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);

mean_global_average_info_handle=subplot(1,4,4);
ylabel('average global information','FontSize',fontsize)
xlabel('saccade','FontSize',fontsize)
set(gca,'fontsize',fontsize);

%ColOrd = get(gca,'ColorOrder');
%[m,n] = size(ColOrd);
h1s=[];
h2s=[];
h3s=[];
h4s=[];


colors{1}=[0 1 1];
colors{2}=[0 0 1];
colors{3}=[1 0.5490 0];
colors{4}=[1 0 0];

legend_string={'foveal fov-90º','foveal fov-135º','cartesian fov-90º','cartesian fov-135º'};

sigma_index=chosen_alhpa;

%plot regrets
subplot(regret_handle)
hold on

y=reshape(cumulative_regret_logpolar(1,sigma_index,:,:),size(cumulative_regret_logpolar,3),size(cumulative_regret_logpolar,4));
h1=plot(mean(y)','Color',colors{1});
h1s=[h1s;h1];

y=reshape(cumulative_regret_logpolar(2,sigma_index,:,:),size(cumulative_regret_logpolar,3),size(cumulative_regret_logpolar,4));
h1=plot(mean(y)','Color',colors{2});
h1s=[h1s;h1];

y=reshape(cumulative_regret_cartesian(1,sigma_index,:,:),size(cumulative_regret_cartesian,3),size(cumulative_regret_cartesian,4));
plot(mean(y)','Color',colors{3},'LineStyle','--');

y=reshape(cumulative_regret_cartesian(2,sigma_index,:,:),size(cumulative_regret_cartesian,3),size(cumulative_regret_cartesian,4));
plot(mean(y)','Color',colors{4},'LineStyle','--');

%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
xlim([1 considered_iterations]);
%plot average local information
subplot(gap_handle)
hold on

y=reshape(gap_data_logpolar(1,sigma_index,:,:),size(gap_data_logpolar,3),size(gap_data_logpolar,4));
h2=plot(mean(y)','Color',colors{1});
h2s=[h2s;h2];

y=reshape(gap_data_logpolar(2,sigma_index,:,:),size(gap_data_logpolar,3),size(gap_data_logpolar,4));
h2=plot(mean(y)','Color',colors{2});
h2s=[h2s;h2];

y=reshape(gap_data_cartesian(1,sigma_index,:,:),size(gap_data_cartesian,3),size(gap_data_cartesian,4));
plot(mean(y)','Color',colors{3},'LineStyle','--');

y=reshape(gap_data_cartesian(2,sigma_index,:,:),size(gap_data_cartesian,3),size(gap_data_cartesian,4));
plot(mean(y)','Color',colors{4},'LineStyle','--');

legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
xlim([1 considered_iterations]);

%plot average local information
subplot(mean_local_average_info_handle)
hold on

y=reshape(local_average_info_logpolar(1,sigma_index,:,:),size(local_average_info_logpolar,3),size(local_average_info_logpolar,4));
h3=plot(mean(y)','Color',colors{1});
h3s=[h3s;h3];

y=reshape(local_average_info_logpolar(2,sigma_index,:,:),size(local_average_info_logpolar,3),size(local_average_info_logpolar,4));
h3=plot(mean(y)','Color',colors{2});
h3s=[h3s;h3];

y=reshape(local_average_info_cartesian(1,sigma_index,:,:),size(local_average_info_cartesian,3),size(local_average_info_cartesian,4));
plot(mean(y)','Color',colors{3},'LineStyle','--');

y=reshape(local_average_info_cartesian(2,sigma_index,:,:),size(local_average_info_cartesian,3),size(local_average_info_cartesian,4));
plot(mean(y)','Color',colors{4},'LineStyle','--');

%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
xlim([1 considered_iterations]);


%plot average global information
subplot(mean_global_average_info_handle)
hold on

y=reshape(global_average_info_logpolar(1,sigma_index,:,:),size(global_average_info_logpolar,3),size(global_average_info_logpolar,4));
h4=plot(mean(y)','Color',colors{1});
h4s=[h4s;h4];

y=reshape(global_average_info_logpolar(2,sigma_index,:,:),size(global_average_info_logpolar,3),size(global_average_info_logpolar,4));
h4=plot(mean(y)','Color',colors{2});
h4s=[h4s;h4];

y=reshape(global_average_info_cartesian(1,sigma_index,:,:),size(global_average_info_cartesian,3),size(global_average_info_cartesian,4));
plot(mean(y)','Color',colors{3},'LineStyle','--');

y=reshape(global_average_info_cartesian(2,sigma_index,:,:),size(global_average_info_cartesian,3),size(global_average_info_cartesian,4));
plot(mean(y)','Color',colors{4},'LineStyle','--');

%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
xlim([1 considered_iterations]);









%% Box plots
colors=cell(4,alphas);
for i=1:alphas
    colors{1,i}=[0 1 1];
    colors{2,i}=[0 0 1];
    colors{3,i}=[1 0.5490 0];
    colors{4,i}=[1 0 0];
end


figure(2)
set(gcf, 'Color', [1,1,1]);
%
% instantaneous_reward_error_bar_logpolar_handle=subplot(1,4,1);
% title('average reward per saccade')

gap_error_bar_handle=subplot(1,4,1);

local_average_info_error_bar_logpolar_handle=subplot(1,4,2);

average_regret_bar_logpolar_handle=subplot(1,4,3);

global_average_info_error_bar_logpolar_handle=subplot(1,4,4);

% change position to be below x-axis by 8%
% offset=0.0
% set(h, 'Units', 'Normalized');
% pos = get(h, 'Position');
% set(h, 'Position', pos + [-offset, 0, 0]);

% subplot(instantaneous_reward_error_bar_logpolar_handle)
% aboxplot(reward_data,colors,'labels',alphas_string); % Advanced box plot
% xlabel('$\alpha$','Interpreter','LaTex'); % Set the X-axis
% plotTickLatex2D();
% legend(legend_string,'Location','southeast'); % Add a legend

subplot(average_regret_bar_logpolar_handle)
aboxplot(regret_data,colors,'labels',alphas_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
h=ylabel('average cumulative regret per saccade','FontSize',fontsize);
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);

subplot(gap_error_bar_handle)
aboxplot(gap_data,colors,'labels',alphas_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis
plotTickLatex2D('FontSize',fontsize);
legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
h=ylabel('average gap reduction per saccade','FontSize',fontsize)
set(h, 'Units', 'Normalized', 'Position', [-0.2, 0.5, 0]);

subplot(local_average_info_error_bar_logpolar_handle)
aboxplot(local_info_data,colors,'labels',alphas_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
h=ylabel('mean average local information per saccade','FontSize',fontsize)
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);

subplot(global_average_info_error_bar_logpolar_handle)
aboxplot(global_info_data,colors,'labels',alphas_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
h=ylabel('mean average global information per saccade','FontSize',fontsize);
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);







figure(3)
set(gcf, 'Color', [1,1,1]);





gap_error_bar_lines_handle=subplot(1,4,1);

local_average_info_error_bar_logpolar_lines_handle=subplot(1,4,2);

average_regret_bar_logpolar_lines_handle=subplot(1,4,3);

global_average_info_error_bar_logpolar_lines_handle=subplot(1,4,4);

% change position to be below x-axis by 8%
% offset=0.0
% set(h, 'Units', 'Normalized');
% pos = get(h, 'Position');
% set(h, 'Position', pos + [-offset, 0, 0]);

% subplot(instantaneous_reward_error_bar_logpolar_handle)
% aboxplot(reward_data,colors,'labels',alphas_string); % Advanced box plot
% xlabel('$\alpha$','Interpreter','LaTex'); % Set the X-axis
% plotTickLatex2D();
% legend(legend_string,'Location','southeast'); % Add a legend
n=size(gap_data{1},2)-2;
subplot(average_regret_bar_logpolar_lines_handle)
mean_ei_pi=[];
for i=1:size(regret_data,2)
    Y=regret_data{i};
    mean=[];
    
    for j=1:size(Y,2)
        [q1 q2 q3 fu fl ou ol outliers_indices] = quartile(Y(:,j));
        
        data_without_outliers=Y(:,j);
        data_without_outliers(outliers_indices)=[];
        u = nanmean(data_without_outliers);
        
        [q1 q2 q3 fu fl ou ol outliers_indices] = quartile(data_without_outliers);
        data_without_outliers(outliers_indices)=[];
        
        if(j>size(Y,2)-2)
            mean_ei_pi=[mean_ei_pi nanmean(data_without_outliers)];
        else
            mean = [mean nanmean(data_without_outliers)];
        end
    end
    hold on
    plot(mean,'color',colors{i})
   
    hold off
end
mean_pi=repmat(mean_ei_pi(1:2:end),5,1);
mean_ei=repmat(mean_ei_pi(2:2:end),5,1);
for i=1:size(mean_pi,2)
    hold on
    plot(mean_pi(:,i),'color',colors{i},'LineStyle','--')
    plot(mean_ei(:,i),'color',colors{i},'LineStyle',':')
    hold off
end

%aboxplot(regret_data,colors,'labels',alphas_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
set(gca,'XTick',1:n);
set(gca,'XTickLabel',alphas_string);
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
h=ylabel('average cumulative regret per saccade','FontSize',fontsize);
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);



mean_ei_pi=[];
subplot(gap_error_bar_lines_handle)
for i=1:size(gap_data,2)
    Y=gap_data{i};
    mean=[];
    for j=1:size(Y,2)
        [q1 q2 q3 fu fl ou ol outliers_indices] = quartile(Y(:,j));
        
        data_without_outliers=Y(:,j);
        data_without_outliers(outliers_indices)=[];
        u = nanmean(data_without_outliers);
        
        [q1 q2 q3 fu fl ou ol outliers_indices] = quartile(data_without_outliers);
        data_without_outliers(outliers_indices)=[];
        if(j>size(Y,2)-2)
            mean_ei_pi=[mean_ei_pi nanmean(data_without_outliers)];
        else
            mean = [mean nanmean(data_without_outliers)];
        end
    end
    hold on
    plot(mean,'color',colors{i})
    hold off
end
mean_pi=repmat(mean_ei_pi(1:2:end),5,1);
mean_ei=repmat(mean_ei_pi(2:2:end),5,1);
for i=1:size(mean_pi,2)
    hold on
    plot(mean_pi(:,i),'color',colors{i},'LineStyle','--')
    plot(mean_ei(:,i),'color',colors{i},'LineStyle',':')
    hold off
end
set(gca,'XTick',1:n);
set(gca,'XTickLabel',alphas_string);
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis
plotTickLatex2D('FontSize',fontsize);
legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
h=ylabel('average gap reduction per saccade','FontSize',fontsize)
set(h, 'Units', 'Normalized', 'Position', [-0.2, 0.5, 0]);


mean_ei_pi=[];

subplot(local_average_info_error_bar_logpolar_lines_handle)
for i=1:size(local_info_data,2)
    Y=local_info_data{i};
    mean=[];
    for j=1:size(Y,2)
        [q1 q2 q3 fu fl ou ol outliers_indices] = quartile(Y(:,j));
        
        data_without_outliers=Y(:,j);
        data_without_outliers(outliers_indices)=[];
        u = nanmean(data_without_outliers);
        
        [q1 q2 q3 fu fl ou ol outliers_indices] = quartile(data_without_outliers);
        data_without_outliers(outliers_indices)=[];
        if(j>size(Y,2)-2)
            mean_ei_pi=[mean_ei_pi nanmean(data_without_outliers)];
        else
            mean = [mean nanmean(data_without_outliers)];
        end
    end
    hold on
    plot(mean,'color',colors{i})
    hold off
end
mean_pi=repmat(mean_ei_pi(1:2:end),5,1);
mean_ei=repmat(mean_ei_pi(2:2:end),5,1);
for i=1:size(mean_pi,2)
    hold on
    plot(mean_pi(:,i),'color',colors{i},'LineStyle','--')
    plot(mean_ei(:,i),'color',colors{i},'LineStyle',':')
    hold off
end
set(gca,'XTick',1:n);
set(gca,'XTickLabel',alphas_string);
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
h=ylabel('mean average local information per saccade','FontSize',fontsize)
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);


mean_ei_pi=[];

subplot(global_average_info_error_bar_logpolar_lines_handle)
for i=1:size(global_info_data,2)
    Y=global_info_data{i};
    mean=[];
    for j=1:size(Y,2)
        [q1 q2 q3 fu fl ou ol outliers_indices] = quartile(Y(:,j));
        
        data_without_outliers=Y(:,j);
        data_without_outliers(outliers_indices)=[];
        u = nanmean(data_without_outliers);
        
        [q1 q2 q3 fu fl ou ol outliers_indices] = quartile(data_without_outliers);
        data_without_outliers(outliers_indices)=[];
        if(j>size(Y,2)-2)
            mean_ei_pi=[mean_ei_pi nanmean(data_without_outliers)];
        else
            mean = [mean nanmean(data_without_outliers)];
        end
    end
    hold on
    plot(mean,'color',colors{i})
    hold off
end

mean_pi=repmat(mean_ei_pi(1:2:end),5,1);
mean_ei=repmat(mean_ei_pi(2:2:end),5,1);
for i=1:size(mean_pi,2)
    hold on
    plot(mean_pi(:,i),'color',colors{i},'LineStyle','--')
    plot(mean_ei(:,i),'color',colors{i},'LineStyle',':')
    hold off
end
set(gca,'XTick',1:n);
set(gca,'XTickLabel',alphas_string);
xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
plotTickLatex2D('FontSize',fontsize);
%legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
h=ylabel('mean average global information per saccade','FontSize',fontsize);
set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);




% trim=25;
% figure(4)
% set(gcf, 'Color', [1,1,1]);
% %
% % instantaneous_reward_error_bar_logpolar_handle=subplot(1,4,1);
% % title('average reward per saccade')
%
% average_regret_bar_logpolar_handle=subplot(1,4,1);
%
% gap_error_bar_handle=subplot(1,4,2);
% %
% local_average_info_error_bar_logpolar_handle=subplot(1,4,3);
% % ylabel('average local information per saccade','FontSize',fontsize)
%
%
% global_average_info_error_bar_logpolar_handle=subplot(1,4,4);
%
% subplot(average_regret_bar_logpolar_handle)
% bar(reshape((trimmean(cell2mat(global_info_data),trim,1)),9,4));
% xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
% plotTickLatex2D('FontSize',fontsize);
% legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
% h=ylabel('average regret per saccade','FontSize',fontsize);
% set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);
%
% subplot(gap_error_bar_handle)
% bar(reshape((trimmean(cell2mat(gap_data),trim,1)),9,4));
% %aboxplot(gap_data,colors,'labels',alphas_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
% xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis
% plotTickLatex2D('FontSize',fontsize);
% legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
% h=ylabel('average gap per saccade','FontSize',fontsize)
% set(h, 'Units', 'Normalized', 'Position', [-0.2, 0.5, 0]);
%
% subplot(local_average_info_error_bar_logpolar_handle)
% bar(reshape((trimmean(cell2mat(local_info_data),trim,1)),9,4));
% %aboxplot(local_info_data,colors,'labels',alphas_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
% xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
% plotTickLatex2D('FontSize',fontsize);
% legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
% h=ylabel('average local information per saccade','FontSize',fontsize)
% set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);
%
% subplot(global_average_info_error_bar_logpolar_handle)
% bar(reshape((trimmean(cell2mat(global_info_data),trim,1)),9,4));
% %aboxplot(global_info_data,colors,'labels',alphas_string,'Interpreter','LaTex','FontSize',fontsize); % Advanced box plot
% xlabel('$\alpha$','Interpreter','LaTex','FontSize',fontsize); % Set the X-axis label
% plotTickLatex2D('FontSize',fontsize);
% legend(legend_string,'Location','southeast','FontSize',fontsize); % Add a legend
% h=ylabel('average global information per saccade','FontSize',fontsize);
% set(h, 'Units', 'Normalized', 'Position', [-0.18, 0.5, 0]);
%









%export_fig box_plots_ucb -pdf






