clc
close all
clear all

debug=0;
do_plots=0;
distances=0.2:0.025:0.225;
total_distances=length(distances);
vergences=0.0:0.01:pi/4;
total_vergences=length(vergences);

%% Create stereoscopic vision system
% resolution
M=200;
N=200;
vc=M/2.0;
uc=N/2.0;
B=0.05;
f=0.022; % ~22 mm in humans
%horizontal_fov = 2 atan(0.5 width / focallength) 
horizontal_fov=90*pi/180;
vertical_fov=90*pi/180;

au=0.5*N/(tan(horizontal_fov/2.0)); % length in pixels
av=0.5*M/(tan(horizontal_fov/2.0));

%kv=1/f * 25;
%ku=1/f * 25;
%au=f*ku; % focal length in pixels
%av=f*kv; % focal length in pixels

%horizontal_fov=2*atan(0.5*N/au);
%vertical_fov=2*atan(0.5*N/av);

% intrinsics
K_LEFT = [au 0 uc;
    0 av vc;
    0 0 1.0];

K_RIGHT = [au 0 uc;
    0 av vc;
    0 0 1];
% fovel stereo parameters
NRINGS=M;
NSECTORS=M;
RMIN=1;
RMAX=min(M,N);
xc=N/2;
yc=M/2;
interp=1;
sp=0;
full=0;
uncertainty_lower_bound=1.0;
L=4.0;
alpha=0.3;
ki=10.0;
beta=2.0;
scaling_factor=0.1;
number_of_disparities=128;
min_disparity=0;
sad_window_size=3;
pre_filter_cap=63;
P1=768;
P2=1536;
uniqueness_ratio=15;
speckle_window_size=50;
speckle_range=16;
disp_12_max_diff=1;
full_dp=0;

%% Initialize foveal stereo object
clear foveal
foveal = foveal_interface(...
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
    uncertainty_lower_bound,...
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
    world_figure=figure(1);
    uncertainty_figure=figure(2);
    images_figure=figure(3);
end

%% Do tests
total_iterations=total_distances*total_vergences;
for distance_index=1:total_distances
    Zw=distances(distance_index);
    optimal_vergence=2*atan2(B/2,Zw); %fixation point

%     closest_vergence_index=1;
%     closest_vergence_diff=abs(vergences(1)-optimal_vergence);
%     for i=2:length(vergences)
%         diff=abs(vergences(i)-optimal_vergence);
%         if abs(vergences(i)-optimal_vergence)<closest_vergence_diff
%             closest_vergence_diff=diff;
%             closest_vergence_index=i;
%         end
%     end
%     vergences(closest_vergence_index)=optimal_vergence;
    
    optimal_vergences(distance_index)=optimal_vergence;
    
    for vergence_index=1:total_vergences
        iteration=vergence_index+(distance_index-1)*total_vergences;
        sprintf('iteration %d of %d (%f percent)', iteration, total_iterations,double(iteration/total_iterations)*100.0)
        
        %% Create 3D world
        disp('create 3D world')
        Zw=distances(distance_index);
        vergence=vergences(vergence_index)
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
        
        % camera matrices
        H=[1 0 0 0;
            0 1 0 0;
            0 0 1 0];
        P_LEFT  = K_LEFT * H * E_LEFT;
        P_RIGHT = K_RIGHT* H * E_RIGHT;
        
        %define the plane in world coordinates
        P_0_W=[0 0 Zw 1]; % origin
        n_0_W=[0 0 -1.0000]; % normal
        
        %transform to the left camera coordinates
        P_0_C_LEFT=E_LEFT*P_0_W';
        n_0_C_LEFT=E_LEFT(1:3,1:3)*n_0_W';
        aux_LEFT=dot(P_0_C_LEFT(1:3),n_0_C_LEFT(1:3));
        
        %transform to right coordinates
        P_0_C_RIGHT=E_RIGHT*P_0_W';
        n_0_C_RIGHT=E_RIGHT(1:3,1:3)*n_0_W';
        aux_RIGHT=dot(P_0_C_RIGHT(1:3),n_0_C_RIGHT(1:3));
        
        world_points_left=zeros(M,N,3); % world points seen from left camera
        world_points_right=zeros(M,N,3); % world points seen from right camera
        
        world_points_common=[];
        
        l_image=double(zeros(M,N,2));
        r_image=double(zeros(M,N,2));
        
        correspondences_r=-1*ones(M,N);
        correspondences_c=-1*ones(M,N);
        for r=1:M
            for c=1:N
                l_image(r,c,1)=r;
                l_image(r,c,2)=c;
                I=[(c-uc)/au (r-vc)/av 1];
                Zc=aux_LEFT/(dot(I,n_0_C_LEFT(1:3)));
                
                % Compute index
                i=c+(r-1)*N;
                
                % Compute world coordinate seen from left camera
                world_points_left_h=E_LEFT\[(c-uc)*Zc/au (r-vc)*Zc/av Zc 1]';
                world_points_left(r,c,:)=world_points_left_h(1:3);
                
                % Project on right camera
                r_camera_points_h=P_RIGHT*world_points_left_h;
                r_camera_points_h(:)=r_camera_points_h(:)./r_camera_points_h(3);
                if r_camera_points_h(1)>=1&&r_camera_points_h(1)<=N && r_camera_points_h(2)>=1 && r_camera_points_h(2)<=M
                    world_points_common=[world_points_common; world_points_left(r,c,:)];
                    correspondences_r(r,c)=round(r_camera_points_h(2));
                    correspondences_c(r,c)=round(r_camera_points_h(1));
                end
                
                Zc=aux_RIGHT/(dot(I,n_0_C_RIGHT(1:3)));
                
                % Compute world coordinate seen from right camera
                world_points_right_h=E_RIGHT\[(c-uc)*Zc/au (r-vc)*Zc/av Zc 1]';
                world_points_right(r,c,:)=world_points_right_h(1:3);
                
                % Project on left camera
                l_camera_points_h=P_LEFT*world_points_right_h;
                l_camera_points_h(:)=l_camera_points_h(:)./l_camera_points_h(3);
                
                if l_camera_points_h(1)>=1&&l_camera_points_h(1)<=N && l_camera_points_h(2)>=1 && l_camera_points_h(2)<=M
                    r_image(r,c,1)=l_camera_points_h(2); % row
                    r_image(r,c,2)=l_camera_points_h(1); % column
                    
                    world_points_common=[world_points_common; world_points_right(r,c,:)];
                end
            end
        end
        
        %% Compute uncertainty
        disp('get uncertainty');
        tic
        
        transf_r_to_l=(E_RIGHT)*inv(E_LEFT);
        
        [image_left_rectified,...
            image_right_rectified,...
            stereo_rectification_map1_left,...
            stereo_rectification_map2_left,...
            stereo_rectification_map1_right,...
            stereo_rectification_map2_right]=rectify_stereo(foveal,l_image,r_image,transf_r_to_l(1:3,1:3)',transf_r_to_l(1:3,4)');
        
        % RECOMPUTE DISPARITY AFTER RECTIFICATION!!!
        
        disparities_rectified=-1*ones(M,N);
        
        for r=1:M
            for c_left=1:N
                c_right_best_index=1;
                best_disparity=-1;
                for c_right=2:N
                    disparity=c_left-c_right;
                    % look for correspondence
                    if abs(image_left_rectified(r,c_left,2)-image_right_rectified(r,c_right,2))<abs(image_left_rectified(r,c_left,2)-image_right_rectified(r,c_right_best_index,2)) &&...
                            abs(image_left_rectified(r,c_left,2)-image_right_rectified(r,c_right,2))<1
                        best_disparity=disparity;
                        c_right_best_index=c_right;
                        break;
                    end
                end
                disparities_rectified(r,c_left)=best_disparity;
            end
        end
        good_disparities_pixels_number=sum(sum(disparities_rectified>0));
        good_pixels(distance_index,vergence_index)=good_disparities_pixels_number;
        if good_disparities_pixels_number==0
            continue
        end
        transform=inv(E_LEFT);
        transform(1:3,1:3)=[1 0 0; 0 1 0; 0 0 1];
        [covariance_matrices, information_matrices, covariance_norms, information_norms, mean_3d, real_3d] =get_uncertainty(foveal,transform',disparities_rectified');
        toc

        total_information(distance_index,vergence_index)=sum(information_norms(~isnan(information_norms)));
        average_information(distance_index,vergence_index)=total_information(distance_index,vergence_index)/(M*N);
        max_information(distance_index,vergence_index)=max(information_norms(~isnan(information_norms)));
               
        %% Do plots
        if do_plots==1
            
            
            figure(images_figure)
            subplot(3,3,1)
            imshow(uint8(256*l_image(:,:,2)/max(max(l_image(:,:,2)))))
            title('left image')
            
            subplot(3,3,2)
            imshow(uint8(256*r_image(:,:,2)/max(max(r_image(:,:,2)))))
            title('right image')
            
            subplot(3,3,4)
            imshow(uint8(256*image_left_rectified(:,:,2)/max(max(image_left_rectified(:,:,2)))));
            title('left rectified image')
            
            subplot(3,3,5)
            imshow(uint8(256*image_right_rectified(:,:,2)/max(max(image_right_rectified(:,:,2)))));
            title('right rectified image')
            
            subplot(3,3,6)
            imshow(uint8(256*disparities_rectified(:,:)/max(max(disparities_rectified))))
            title('disparity rectified image')
            
            %% Plot stereo system
            figure(world_figure)
            clf(world_figure)
            s1_handle=subplot(1,1,1);
            l_image_dummy=zeros(M,N);
            r_image_dummy=zeros(M,N);
            
            RetinalCortical(foveal,s1_handle, E_LEFT, uint8(l_image_dummy));
            RetinalCortical(foveal,s1_handle, E_RIGHT, uint8(r_image_dummy));
            
            az = -30.0;
            el = 20;
            view(az, el);
           
            xlabel('z'); ylabel('x'); zlabel('y');
            hold on
            information_plot=log(sqrt(information_norms));
            uncertainty_mesh=surf(mean_3d(:,:,3),-mean_3d(:,:,1),-mean_3d(:,:,2),information_plot);
            set(uncertainty_mesh,'EdgeColor','none')
            colormap parula
            colorbar_handle = colorbar;
            %set(colorbar_handle, 'Position', [0 0 0 .05]);
            ylabel(colorbar_handle, 'information matrix volume (log)')
            caxis([min(min(information_plot)) max(max(information_plot))])
            axis equal
%             pdfFileName='log_information_3D_plane.pdf';
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


figure_handle=figure(11)
%subplot(1,3,1)
[X,Y] = meshgrid(distances,vergences*180/pi);
sum_mesh=surf(X,Y,log(total_information)');
set(sum_mesh,'EdgeColor','none')
colormap parula
colorbar_handle = colorbar;
ylabel(colorbar_handle, 'log information')
caxis([min(min(log(total_information(~isinf(log(total_information)))))) max(max(log(total_information)))])

xlim([distances(1) distances(end)]);
ylim([vergences(1)*180/pi vergences(end)*180/pi]);
xlabel('distance'); ylabel('vergence angle'); zlabel('log information')
title('total information')
pdfFileName='teste.pdf';
dpi=600;
save2pdf(pdfFileName,figure_handle,dpi)
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

