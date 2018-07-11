clc
close all
clear all

debug=1;
do_plots=1;
distances=0.5:0.01:0.5;
%distances=[0.1 0.25 0.5 0.75 1.0];
total_distances=length(distances);
vergences=0:0.5:90;
vergences=vergences*pi/180;
total_vergences=length(vergences);

%% Create stereoscopic vision system
% resolution
M=200;
N=200;
vc=M/2.0;
uc=N/2.0;
B=0.065;
f=0.022; % ~22 mm in humans
%horizontal_fov = 2 atan(0.5 width / focallength) 
horizontal_fov=90*pi/180;
vertical_fov=90*pi/180;

au=0.5*N/(tan(horizontal_fov/2.0)); % length in pixels
av=0.5*M/(tan(horizontal_fov/2.0));

% intrinsics
K_LEFT = [au 0 uc;
    0 av vc;
    0 0 1.0];

K_RIGHT = [au 0 uc;
    0 av vc;
    0 0 1];

% fovel stereo parameters
RMIN=0.5;
RMAX=0.5*min(M,N);
syms NRINGS

eq=RMAX^2*pi/NRINGS==2*pi/(exp(log(RMAX/RMIN)/NRINGS)-1);
NRINGS=abs(round(double(solve(eq,NRINGS))));
NSECTORS=abs(round(RMAX^2*pi/NRINGS));
%a=exp(log(pmax/po)/R);
% NSECTORS=(2*pi*RMAX/sqrt(2*log(RMAX/RMIN)));
% NRINGS=round(log(RMAX/RMIN)*NSECTORS/(2*pi));
% NSECTORS=round(NSECTORS);
xc=N/2;
yc=M/2;
interp=1;
sp=0;
full=0;
uncertainty_lower_bound=0.0;
L=4.0;
alpha=0.25;
ki=3.0;
beta=2.0;
scaling_factor=1;
number_of_disparities=N;
min_disparity=0;
sad_window_size=7;
pre_filter_cap=63;
P1=768;
P2=1536;
uniqueness_ratio=15;
speckle_window_size=50;
speckle_range=16;
disp_12_max_diff=0;
full_dp=0;
fovea_rows=10;
fovea_columns=10;
%% Initialize foveal stereo object
clear foveal
foveal = foveated_stereo(...
    K_LEFT',...
    K_RIGHT',...
    N,...
    M,...
    xc,...
    yc,...
    NRINGS,...
    NSECTORS,...
    RMIN,...
    RMAX,...
    interp,...
    sp,...
    full,...
    fovea_rows,...
    fovea_columns,...
    L,...
    alpha,...
    ki,...
    beta,...
    scaling_factor,...
    number_of_disparities,...
    pre_filter_cap,...
    sad_window_size,...
    P1,...
    P2,...
    min_disparity,...
    uniqueness_ratio,...
    speckle_window_size,...
    speckle_range,...
    disp_12_max_diff,...
    full_dp);

%% allocate data arrays memory
total_information=zeros(total_distances,total_vergences);
average_information=zeros(total_distances,total_vergences);
max_information=zeros(total_distances,total_vergences);

average_variance=zeros(total_distances,total_vergences);
max_variance=zeros(total_distances,total_vergences);

optimal_vergences=zeros(1,total_distances);
good_pixels=zeros(total_distances,total_vergences);

%% create figures
if do_plots==1
    world_figure=figure('units','normalized','outerposition',[0 0 1 1]);
    uncertainty_figure=figure(2);
end

%% Do tests
total_iterations=total_distances*total_vergences;
for distance_index=1:total_distances
    Zw=distances(distance_index);    
    optimal_vergences(distance_index)=2*atan2(B/2,Zw);
    
    for vergence_index=1:total_vergences
        iteration=vergence_index+(distance_index-1)*total_vergences;
        string=sprintf('iteration %d of %d (%f percent)', iteration, total_iterations,double(iteration/total_iterations)*100.0);
        disp(string);
        %% Create 3D world
        Zw=distances(distance_index);
        vergence=vergences(vergence_index);
        %vergence=optimal_vergence;
        version=0;
        %extrinsics (WORLD TO CAMERA COORDINATES)
        E_LEFT = [1 0 0 B/2;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
        
        E_RIGHT = [1 0 0 -B/2;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];
        
        E_LEFT(1:3,1:3)=angle2dcm(0,version+vergence/2.0,0);
        E_RIGHT(1:3,1:3)=angle2dcm(0,version-vergence/2.0,0);      
        
        l_image=double(zeros(M,N,2));
        r_image=double(zeros(M,N,2));
        
        %% Compute uncertainty
        disp('get uncertainty');
        tic
        
        transf_r_to_l=(E_RIGHT)*inv(E_LEFT);
        
        [image_left_rectified,...
            image_right_rectified,...
            H1,...
            H2,...
            stereo_rectification_map1_left,...
            stereo_rectification_map2_left,...
            stereo_rectification_map1_right,...
            stereo_rectification_map2_right,...
            cameraMatrix1Rectified,...
            cameraMatrix2Rectified,...
            translation,...
            Q]=rectify_stereo(foveal,l_image,r_image,transf_r_to_l(1:3,1:3)',transf_r_to_l(1:3,4)',K_LEFT',K_RIGHT');

        % RECOMPUTE DISPARITY AFTER RECTIFICATION!!!
        disparity=cameraMatrix1Rectified(1,1)*B/Zw;
        disparities_rectified=-1*ones(M,N);
        
        for r=1:M
            for c=1+round(disparity):N
                disparities_rectified(r,c)=disparity;
            end
        end
        good_disparities_pixels_number=sum(sum(disparities_rectified>0));
        good_pixels(distance_index,vergence_index)=good_disparities_pixels_number;
        if good_disparities_pixels_number==0
            continue
        end

        transform=inv(E_LEFT);
        transform(1:3,1:3)=[1 0 0; 0 1 0; 0 0 1];
        [covariance_matrices,...
            information_matrices,...
            information_norms,...
            mean_3d,...
            real_3d,...
            mean_sigma_points,...
            mean_sigma_points_rectified_1,...
            mean_sigma_points_rectified_2]=get_uncertainty(foveal,...
                transform',...
                disparities_rectified',...
                H1',...
                H2',...
                stereo_rectification_map1_left,...
                stereo_rectification_map2_left,...
                stereo_rectification_map1_right,...
                stereo_rectification_map2_right,...
                translation,...
                Q');
        toc

        %information_norms=sqrt(information_norms); % STD DEVIATION INSTEAD OF VARIANCE
        %covariance_norms=sqrt(covariance_norms); % STD DEVIATION INSTEAD OF VARIANCE
        if sum(sum(~isnan(log(information_norms))))==0
            disp('oups');
            continue
        end
        total_information(distance_index,vergence_index)=sum((log(information_norms(~isnan((information_norms))))));
        average_information(distance_index,vergence_index)=total_information(distance_index,vergence_index)/(NSECTORS*NRINGS);
        max_information(distance_index,vergence_index)=max((log(information_norms(~isnan((information_norms))))));
        
        
        
        %% Do plots
        if do_plots==1
            %% Plot stereo system
            figure(world_figure)
            clf(world_figure)
            %s1_handle=subplot(1,1,1);
            l_image_dummy=zeros(M,N);
            r_image_dummy=zeros(M,N);
            RetinalCortical(world_figure, E_LEFT, uint8(l_image_dummy),uint8(l_image_dummy),uint8(l_image_dummy));
            RetinalCortical(world_figure, E_RIGHT, uint8(r_image_dummy),uint8(l_image_dummy),uint8(l_image_dummy));
            
            az = 30.0;
            el = 20;
            view(az, el);
           
            xlabel('z','FontSize',10); ylabel('x','FontSize', 10); zlabel('y','FontSize', 10);
            hold on
            information_plot=log(sqrt(information_norms));
            
            order_ = [3 1 2] ; % 2nd goes to 5th, etc.

            real_3d_plot = real_3d(:,:,order_);
            real_3d_plot(:,:,2)=-real_3d_plot(:,:,2);
            real_3d_plot(:,:,3)=-real_3d_plot(:,:,3);
            showPointCloud(real_3d_plot,information_plot);
            colormap parula
            colorbar_handle = colorbar;
            set(colorbar_handle, 'Position', [0.75 0.15 .02 .8150])
            ylabel(colorbar_handle, 'log-information','FontSize', 24)
            
            caxis([min(min(information_plot)) max(max(information_plot))])
            axis equal
            set(gcf, 'Color', [1,1,1]);
            set(gcf,'defaultaxesposition',[0 0 1 1]) 
            tolerance_=0.01;
            axis([-0.02 max(nanmax((real_3d_plot(:,:,1))))+tolerance_ min(nanmin((real_3d_plot(:,:,2))))-tolerance_ max(nanmax((real_3d_plot(:,:,2))))+tolerance_ min(nanmin((real_3d_plot(:,:,3))))-tolerance_ max(nanmax((real_3d_plot(:,:,3))))+tolerance_])
            drawnow
%             xlabh = get(gca,'XLabel');
%             set(xlabh,'Position',get(xlabh,'Position') + [0.0 0 0])
%             ylabh = get(gca,'YLabel');
%             set(ylabh,'Position',get(ylabh,'Position') + [0 0.0 0.0])
            
            %rgb2cm
            %set(gcf,'Renderer','painters')
            %pdfFileName='log_information_3D_plane.pdf';
            %print(gcf,pdfFileName,'-dpdf','-r600');
%             dpi=600;
%             save2pdf(pdfFileName,world_figure,dpi)
            


            %         h1 = axes
            %         set(world_figure, 'Ydir', 'reverse')
            %         end
            %         % Your two points
            %         P1_L = (E_LEFT)*[0,0,0,1]'; %
            %         P2 = [0,0,Zw];
            %         P1_R = (E_RIGHT)*[0,0,0,1]';
            %
            %         % % Their vertial concatenation is what you want
            %         pts = [P1_L(1:3)'; P2; ...
            %             P1_R(1:3)'; P2];
            %
            %         % % Alternatively, you could use plot3:
            %         plot3(pts(:,3), -pts(:,1), -pts(:,2),'r')
            %
            % plot covariance matrices
            %             cov_plot_step=10;
            %             for v=0:M-1
            %                 for u=0+disparity:cov_plot_step:N-1
            %                     %if disparities(v+1,u+1)<=-1000
            %                     %    continue
            %                     %end
            %                     if covariance_norms(v+1,u+1)~=covariance_norms(v+1,u+1) || log(sqrt(information_norms(v+1,u+1)))<1
            %                         continue
            %                     end
            %                     cov_test=covariance_matrices((3*v+1):(3*v+3),(3*u+1):(3*u+3));
            %                     i=u+1-disparity+v*(N-disparity);
            %                     pos_test=world_points_h(i,:);
            %                     plot3(pos_test(1), pos_test(2),pos_test(3));
            %                     hold on
            %                     plot_gaussian_ellipsoid(pos_test(1:3), cov_test, 0.1);
            %                 end
            %             end
            
            
            %% Plot uncertainty
            %         figure(uncertainty_figure)
            %         clf(uncertainty_figure)
            %         information_plot=log(sqrt(information_norms));
            %         [X,Y]=meshgrid(1:N,1:M);
            %         information_mesh=contourf(X,Y,log(sqrt(information_norms)));
            %         cmap=colormap % This will show the colour axis
            %         caxis([1 max(max(information_plot))])
            %         xlabel('x'); ylabel('y'); title('information')
            %         daspect([1 1 1]);
            %         title('Uncertainty')
            if debug==1
                k = waitforbuttonpress
            end
        end
    end
end


figure_handle=figure(10)
distance_end_index=length(distances);
vergence_end_index=length(vergences);
%subplot(1,3,1)
[X,Y] = meshgrid(distances(1:distance_end_index),vergences(1:vergence_end_index)*180/pi);
sum_mesh=surf(X,Y,log(total_information(1:distance_end_index,1:vergence_end_index))');
set(sum_mesh,'EdgeColor','none')
colormap parula
colorbar_handle = colorbar;
%ylabel(colorbar_handle, 'log information')
caxis([min(min(log(total_information(~isinf(log(total_information(1:distance_end_index,1:vergence_end_index))))))) max(max(log(total_information(1:distance_end_index,1:vergence_end_index))))])

xlim([distances(1) distances(distance_end_index)]);
ylim([vergences(1)*180/pi vergences(vergence_end_index)*180/pi]);
xlabel('distance'); ylabel('vergence angle'); zlabel('log information')
title('total information')
pdfFileName='total_log_information.pdf';
dpi=600;
%save2pdf(pdfFileName,figure_handle,dpi)
% subplot(1,3,2)
% [X,Y] = meshgrid(distances,vergences*180/pi);
% average_mesh=surf(X,Y,log(average_information)');
% set(average_mesh,'EdgeColor','none')
% xlim([distances(1) distances(end)]);
% ylim([vergences(1)*180/pi vergences(end)*180/pi]);
% xlabel('distance'); ylabel('vergence angle'); zlabel('average log-information');
% title('average log-information')
% 
% subplot(1,3,3)
% [X,Y] = meshgrid(distances,vergences*180/pi);
% max_mesh=surf(X,Y,log(max_information)');
% set(max_mesh,'EdgeColor','none')
% xlim([distances(1) distances(end)]);
% ylim([vergences(1)*180/pi vergences(end)*180/pi]);
% xlabel('distance'); ylabel('vergence angle');
% title('max information')

clear foveal

