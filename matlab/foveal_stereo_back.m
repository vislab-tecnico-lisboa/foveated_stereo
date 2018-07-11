clc
clear all
close all

addpath(genpath('.'))

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
right_image_rgb  = undistortImage(right_image_rgb,right_cam_params);


left_image_gray  = rgb2gray(imread('tsukuba_left.png'));
right_image_gray = rgb2gray(imread('tsukuba_right.png'));


%CROP IMAGES (SHOULD BE SQUARED)
if size(left_image_rgb,1)~=size(left_image_rgb,2)
    c = .5*size(left_image_gray); % center
    w = min( size(left_image_gray) ); % take min dimension
    bottom=ceil([c(2 c(1)]-.5*[w w])+1;
    rect = [ bottom, [w-1 w-1] ];
    left_image_gray = imcrop( left_image_gray, rect); 
    right_image_gray = imcrop(right_image_gray, rect);
end

%%%%%%%%%%%%%%%%%%%%%%%
% Get cortical images %
%%%%%%%%%%%%%%%%%%%%%%%

min_disparity=-40;
max_disparity=40;
horizontal_disparity_step=2;
horizontal_disparities=min_disparity:horizontal_disparity_step:max_disparity;
horizontal_total_disparities=size(horizontal_disparities,2);

sigma=3.0;
q=0.1;
pole=0.8;
focal_distance=1.0;
spherical_angle_bins=360;
M=size(left_image_gray,1)
N=size(left_image_gray,2)
NRINGS=128;
NSECTORS=256;
%NRINGS=360;
%NSECTORS=360;
RMIN=1;
RMAX=min(M,N);
xc=N/2;
yc=M/2;
interp=1;
sp=0;
full=0;

%% Initialize foveal stereo object
clear foveal
foveal = foveal_interface(N, M, xc, yc, NRINGS, NSECTORS, RMIN, RMAX, interp, sp, full, horizontal_disparities, sigma, q, pole, focal_distance, spherical_angle_bins, 0); % Create a class instance

disp('transform images to the cortical domain');
tic
left_image_log_gray =to_cortical(foveal,left_image_gray');
right_image_log_gray=to_cortical(foveal,right_image_gray');
toc

figure_handle=subplot(1,2,1);
RetinalCortical(foveal,left_image_gray,figure_handle,-1.5)
RetinalCortical(foveal,right_image_gray,figure_handle,1.5)


%% debug spatial forward-backward IIR filter
figure
subplot(1,3,1)
imshow(left_image_gray)
filtered_log=filter(foveal, left_image_log_gray', -0.5, 2, 2);
subplot(1,3,2)
imshow(filtered_log)

filtered_cartesian=to_cartesian(foveal,filtered_log');
subplot(1,3,3)
imshow(filtered_cartesian)

%% get cortical warp tables
disp('get cortical warp maps');
tic
[eta, rho]=get_delta_logpolar_warp_maps(foveal);
toc

%% get cortical shifted images
% figure(1)
% set(gcf,'units','normalized','outerposition',[0 0 1 1])
% for i=1:size(horizontal_disparities,2)
%     original_cartesian=to_cartesian(foveal,right_image_log_gray');
%     %subplot(1,3,1); imshow(original_cartesian);
%     bigsubplot(1,3,1,1,0); imshow(original_cartesian);
% 
%     warped_logpolar=warp_logpolar(foveal,right_image_log_gray',i);
%     cartesian_warped=to_cartesian(foveal,warped_logpolar');
%    
%     bigsubplot(1,3,1,2,0); imshow(warped_logpolar);
%     bigsubplot(1,3,1,3,0);  imshow(cartesian_warped);
%     drawnow;   
%end

%%%%%%%%%%%%%%%%%%%
% CORTICAL STEREO %
%%%%%%%%%%%%%%%%%%%

disp('computing disparity maps');
tic
log_disparity_map=compute_disparity_map(foveal, left_image_log_gray', right_image_log_gray');
toc
marker_idx = (log_disparity_map == 1000.0);
log_disparity_map(marker_idx) = max(log_disparity_map(~marker_idx));
log_disparity_map = log_disparity_map-min_disparity;

log_disparity_map=uint8(log_disparity_map);

figure(2)
subplot(1,2,1); imshow(mat2gray(255-log_disparity_map));

disparity_map=to_cartesian(foveal,log_disparity_map');
subplot(1,2,2); imshow(mat2gray(255-disparity_map));

%% debug
figure(3)
disp('computing disparity map matlab');
tic
disparity_map_matlab = disparity(left_image_gray, right_image_gray, 'DisparityRange', [min_disparity max_disparity], 'Method','SemiGlobal');
toc

subplot(1,3,1); imshow(mat2gray(disparity_map_matlab));

disparity_map_matlab =uint8(disparity_map_matlab);
logpolar_disparity_map_matlab=to_cortical(foveal,disparity_map_matlab');
subplot(1,3,2); imshow(mat2gray(logpolar_disparity_map_matlab));
cartesian_disparity_map_matlab=to_cartesian(foveal,logpolar_disparity_map_matlab');
subplot(1,3,3); imshow(mat2gray(cartesian_disparity_map_matlab));

clear foveal

return




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


% figure(3)
% subplot(1,3,1)
% imshow(left_image)
% subplot(1,3,2)
% imshow(disparity_map_image)
% B=logsampback(disparity_map_image,RMIN,RMAX);
% subplot(1,3,3)
% imshow(B)


% Rcb = imref2d(size(left_image))
% tform = affine2d([1 0 0; 0 1 0; 1000 0 1]);
% [out,Rout] = imwarp(left_image,tform);
%
% figure;
% subplot(1,2,1);
% imshow(left_image,Rcb);
% subplot(1,2,2);
% imshow(out,Rout)

