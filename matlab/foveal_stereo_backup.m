clc
clear all
close all
images = dir('Adirondack-perfect/*.png');
left_image_rgb  = imread('Adirondack-perfect/im0.png');
right_image_rgb = imread('Adirondack-perfect/im1.png');

f=fopen('Adirondack-perfect/calib.txt');
l=fgetl(f);
left_cam_params =cameraParameters('IntrinsicMatrix',cell2mat(reshape(textscan(l,'cam0=[%f %f %f; %f %f %f; %f %f %f]'),3,3)));
l=fgetl(f);
right_cam_params=cameraParameters('IntrinsicMatrix',cell2mat(reshape(textscan(l,'cam1=[%f %f %f; %f %f %f; %f %f %f]'),3,3)));

% calibrate images
left_image_rgb   = undistortImage(left_image_rgb,left_cam_params);
right_image_rgb  = undistortImage(right_image_rgb,left_cam_params);
left_image_gray  = rgb2gray(left_image_rgb);
right_image_gray = rgb2gray(right_image_rgb);

%%%%%%%%%%%%%%%%%%%%%%%
% Get cortical images %
%%%%%%%%%%%%%%%%%%%%%%%

sigma=3;
horizontal_disparity_step=10;
horizontal_disparities=-280:horizontal_disparity_step:280;
horizontal_total_disparities=size(horizontal_disparities,2);
M=size(left_image_rgb,1);
N=size(left_image_rgb,2);
%NRINGS=10;
%NSECTORS=36;
NRINGS=100;
NSECTORS=100;
RMIN=10;
RMAX=min(M,N);
xc=N/2;
yc=M/2;
interp=1;
sp=0;
full=1;
clear foveal

%% Initialize foveal stereo object
foveal = foveal_interface(N, M, xc, yc, NRINGS, NSECTORS, RMIN, RMAX, interp, sp, full, horizontal_disparities); % Create a class instance

disp('transform images to the cortical domain');
tic
left_image_log_gray =to_cortical(foveal,left_image_gray');
right_image_log_gray=to_cortical(foveal,right_image_gray');
toc

%figure(1); imshowpair(left_image_gray,right_image_gray,'ColorChannels','red-cyan');
%title('Red-cyan composite view of the stereo images');

% cartesian disparity tables (offline)
disp('computing offline cartesian disparity maps');
tic
cartesian_disparity_maps=zeros(M,N,2,horizontal_total_disparities);

for j=1:N
    for disparity=1:horizontal_total_disparities
        dx=horizontal_disparities(disparity);
        if (j+dx<1)
            %cartesian_disparity_maps(:,j,2,disparity)=Inf; %Horizontal columns
            continue;
        end
        if (j+dx>N)
            %cartesian_disparity_maps(:,j,2,disparity)=Inf; %Horizontal columns
            break
        end
        cartesian_disparity_maps(:,j,1,disparity)=0;
        cartesian_disparity_maps(:,j,2,disparity)=dx; %Horizontal columns
    end
end
toc

%% get cortical warp tables
disp('get cortical warp maps')
tic
%cortical_disparity_maps=logpolar_disparity_maps(RMIN,RMAX,NSECTORS,NRINGS,horizontal_disparities,horizontal_total_disparities);
[eta, rho]=get_delta_logpolar_warp_maps(foveal);
toc


% figure(2)
% for i=2:2:2*horizontal_total_disparities
%     subplot(horizontal_total_disparities,2,i-1);
%     point_density=300;
%     [X, Y]=meshgrid(1:point_density:N,1:point_density:M);
%     quiver(X,...
%         Y,...
%         cartesian_disparity_maps(1:point_density:M,1:point_density:N,2,i/2),...
%         cartesian_disparity_maps(1:point_density:M,1:point_density:N,1,i/2));
%     axis([0,N , 0,M])
%     subplot(horizontal_total_disparities,2,i);
%     point_density=30;
%     [P, THETA]=meshgrid(1:point_density:NRINGS,1:point_density:NSECTORS);
%      
%     quiver(P,...
%         THETA,...
%         rho(1:point_density:NSECTORS,1:point_density:NRINGS,i/2),...
%         eta(1:point_density:NSECTORS,1:point_density:NRINGS,i/2));
%     
%     axis([0,NRINGS,0,NSECTORS])
% end


%% Intuition stuff
%figure(1)

% for i=101:size(left_image_gray,2)
%     figure(1)
% 
%     black_column_cart=right_image_gray;
%     subplot(1,3,1)
%     black_column_cart( (yc-500):(yc+500),(i-100):(i+100))=255;
%     imshow(black_column_cart)
% 
%     subplot(1,3,2)
%     black_column_logpolar=to_cortical(foveal,black_column_cart');
%     imshow(black_column_logpolar)
%     
%     subplot(1,3,3)
%     retinal=warp_logpolar(foveal,right_image_log_gray',1);
%     imshow(retinal)
% 
% end
% 
% return
%% get cortical shifted images
figure
original_cartesian=to_cartesian(foveal,right_image_log_gray');
imshow(original_cartesian)

figure
imshow(right_image_log_gray)
warped_logpolar=warp_logpolar(foveal,right_image_log_gray',1);

% figure
% imshow(original_cartesian);
% 
% figure
% imshow(right_image_log_gray);

% Shifted retinal to cortical
warped_cartesian_logpolar=warp_cartesian_logpolar(foveal,right_image_gray',1);

figure
title('warped logpolar');
imshow(warped_logpolar);
figure
imshow(warped_cartesian_logpolar);

disparity_map=compute_disparity_map(foveal,left_image_log_gray',right_image_log_gray');
disparity_map=to_cartesian(foveal,disparity_map');

disparity_map_matlab = disparity(left_image_gray,right_image_gray);

figure 
imshow(disparity_map);
% figure
% cartesian_logpolar=to_cartesian(foveal,warped_logpolar');
% imshow(cartesian_logpolar);
% figure
% cartesian_cartesian=to_cartesian(foveal,warped_cartesian_logpolar');
% imshow(cartesian_cartesian);




% Shifted cortical to renitnal
% cartesian_cartesian=to_cartesian(foveal,warped_cartesian_logpolar');
% 
% cartesian_logpolar =to_cartesian(foveal,warped_logpolar');
% 

% 
% figure
% imshow(warped_cartesian_logpolar);
% 
% figure
% imshow(cartesian_cartesian);
% 
% figure
% imshow(cartesian_logpolar);

clear foveal

return
%%%%%%%%%%%%%%%%%%%
% CORTICAL STEREO %
%%%%%%%%%%%%%%%%%%%

left_image=left_image_log_gray;
right_image=right_image_log_gray;

%tic
disparity_map = disparity(left_image_gray,right_image_gray);
%disparity_map = disparity(left_image_log_gray,right_image_log_gray);
%toc

disp('computing disparity maps');
correspondences=zeros(NSECTORS,NRINGS,horizontal_total_disparities);

tic
% Horizontal log disparities
for s=1:NSECTORS
    for r=1:NRINGS
        for disparity=1:horizontal_total_disparities
            %if cortical_disparity_maps(s,r,1,disparity)==Inf
            %    continue;
            %end
            dx=horizontal_disparities(disparity);
            dtheta=cortical_disparity_maps(s,r,1,disparity); % delta theta
            dp=cortical_disparity_maps(s,r,2,disparity); % delta rho
            right_theta=round(s+dtheta);
            right_p=round(r+dp);
            if (right_theta<1) || (right_theta>NSECTORS) ||...
                    (right_p <1) || (right_p>NRINGS)
                continue;
            end
            correspondences(s,r,disparity)=...
                double(left_image_log_gray(s,r))-double(right_image_log_gray(right_theta,right_p));
        end
    end
end
toc

correspondence_likelihood_images = gaussmf(correspondences,[sigma 0]);
disparity_map=zeros(NSECTORS,NRINGS);
for s=1:NSECTORS
    for r=1:NRINGS
        [M,index]=max(correspondence_likelihood_images(s,r,:));
        disparity_map(s,r)=horizontal_disparities(index);
    end
end

disparity_map_image=int8((disparity_map-min(horizontal_disparities))  *(256.0/abs(max(horizontal_disparities)-min(horizontal_disparities))));
toc




% tic
% for i=1:M %for each line
%     for j=1:N %for each column
%         % horizontal disparities
%         for d=-horizontal_disparities:1:horizontal_disparities
%             if (j+d<1) || (j+d>N)
%                 continue;
%             end
%             disparity_maps(i,j,d+horizontal_disparities+1)=double(left_image_gray(i,j))-double(left_image_gray(i,j+d));
%         end
%     end
% end
%
% disparity_likelihood_images = gaussmf(disparity_maps,[sigma 0]);
%
% disparity_map=zeros(M,N);
%
% for i=1:M %for each line
%     for j=1:N %for each column
%         [M,index]=max(disparity_likelihood_images(i,j,:));
%         disparity_map(i,j)=index;
%     end
% end
% disparity_map=(disparity_map*256/horizontal_total_disparities)+ceil(horizontal_total_disparities/2);
%
% toc


figure(1)
subplot(1,3,1)
imshow(left_image_gray)

subplot(1,3,2)
imshow(left_image_log_gray)

retinal=to_cartesian(foveal,left_image_log_gray');

subplot(1,3,3)
imshow(retinal)

figure(2)
for i=2:2:2*horizontal_total_disparities
    subplot(horizontal_total_disparities,2,i-1);
    point_density=300;
    [X Y]=meshgrid(1:point_density:N,1:point_density:M);
    quiver(X,...
        Y,...
        cartesian_disparity_maps(1:point_density:M,1:point_density:N,2,i/2),...
        cartesian_disparity_maps(1:point_density:M,1:point_density:N,1,i/2));
    axis([0,N , 0,M])
    subplot(horizontal_total_disparities,2,i);
    point_density=1;
    [P THETA]=meshgrid(1:point_density:NRINGS,1:point_density:NSECTORS);
     
    quiver(P,...
        THETA,...
        rho(1:point_density:NSECTORS,1:point_density:NRINGS,i/2),...
        eta(1:point_density:NSECTORS,1:point_density:NRINGS,i/2));
    
    axis([0,NRINGS,0,NSECTORS])
end


figure(3)
subplot(1,3,1)
imshow(left_image)
subplot(1,3,2)
imshow(disparity_map_image)
B=logsampback(disparity_map_image,RMIN,RMAX);
subplot(1,3,3)
imshow(B)


% Rcb = imref2d(size(left_image))
% tform = affine2d([1 0 0; 0 1 0; 1000 0 1]);
% [out,Rout] = imwarp(left_image,tform);
%
% figure;
% subplot(1,2,1);
% imshow(left_image,Rcb);
% subplot(1,2,2);
% imshow(out,Rout)

