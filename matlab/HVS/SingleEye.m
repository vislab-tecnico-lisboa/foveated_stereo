clc
close all
clear all

images = dir('Bowling2/*.png');
left_image_rgb  = imread('Adirondack-perfect/im0.png');
right_image_rgb = imread('Adirondack-perfect/im1.png');
disparities = parsePfm( 'Adirondack-perfect/disp0.pfm' );
% left_image_rgb  = imread('Bowling2/view1.png');
% right_image_rgb = imread('Bowling2/view5.png');
% disparities = imread( 'Bowling2/disp1.png' );
% disparities=disparities/3.0;


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


%CROP IMAGES (SHOULD BE SQUARED)
if size(left_image_rgb,1)~=size(left_image_rgb,2)
    c = .5*size(left_image_gray); % center
    w = min( size(left_image_gray) ); % take min dimension
    bottom=ceil([c(2) c(1)]-.5*[w w])+1;
    rect = [ bottom, [w-1 w-1] ];
    left_image_gray = imcrop( left_image_gray, rect);
    left_image_rgb = imcrop( left_image_rgb, rect);
    right_image_gray = imcrop( right_image_gray, rect);
    right_image_rgb = imcrop( right_image_rgb, rect);

end

% CROP AGAIN TO GET CIRCULAR
ci = [size(left_image_gray,2)/2, size(left_image_gray,2)/2, size(left_image_gray,2)/2];     % center and radius of circle ([c_row, c_col, r])
[xx,yy] = ndgrid((1:size(left_image_gray,2))-ci(1),(1:size(left_image_gray,2))-ci(2));
mask = uint8((xx.^2 + yy.^2)<ci(3)^2);
croppedImage = uint8(zeros(size(left_image_rgb)));
croppedImage(:,:,1) = left_image_rgb(:,:,1).*mask;
croppedImage(:,:,2) = left_image_rgb(:,:,2).*mask;
croppedImage(:,:,3) = left_image_rgb(:,:,3).*mask;
left_image_rgb=croppedImage;

croppedImage(:,:,1) = right_image_rgb(:,:,1).*mask;
croppedImage(:,:,2) = right_image_rgb(:,:,2).*mask;
croppedImage(:,:,3) = right_image_rgb(:,:,3).*mask;
right_image_rgb=croppedImage;

%imshow(croppedImage);


%% Create stereoscopic vision system
M=size(left_image_rgb,1);
N=size(left_image_rgb,2);
vc=M/2.0;
uc=N/2.0;
B=0.05;
f=0.022; % ~22 mm in humans
%horizontal_fov = 2 atan(0.5 width / focallength)
horizontal_fov=90*pi/180;
vertical_fov=90*pi/180;

%au=0.5*N/(tan(horizontal_fov/2.0)); % length in pixels
%av=0.5*M/(tan(horizontal_fov/2.0));
au=3740;
av=3740;
% intrinsics
K_LEFT = [au 0 uc;
    0 av vc;
    0 0 1.0];

K_RIGHT = [au 0 uc;
    0 av vc;
    0 0 1];

E_LEFT = [1 0 0 B/2;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

E_RIGHT = [1 0 0 -B/2;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];


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

left_image_logpolar = to_cortical(foveal,left_image_gray');
right_image_logpolar = to_cortical(foveal,right_image_gray');



transf_r_to_l=(E_RIGHT)*inv(E_LEFT);
        
[image_left_rectified,...
           image_right_rectified,...
            stereo_rectification_map1_left,...
            stereo_rectification_map2_left,...
            stereo_rectification_map1_right,...
            stereo_rectification_map2_right]=rectify_stereo(foveal,left_image_gray,right_image_gray,transf_r_to_l(1:3,1:3)',transf_r_to_l(1:3,4)');
        

transform=inv(E_LEFT);
transform(1:3,1:3)=[1 0 0; 0 1 0; 0 0 1];
[covariance_matrices, information_matrices, covariance_norms, information_norms, mean_3d, real_3d] =get_uncertainty(foveal,transform',double(disparities)');

hvs_handle=figure(1);

RetinalCortical(hvs_handle, E_LEFT, left_image_rgb, left_image_rgb, left_image_logpolar);
RetinalCortical(hvs_handle, E_RIGHT, right_image_rgb, right_image_rgb, right_image_logpolar);
hold on
sampling_step=round(size(real_3d,1)/100);
p1 = real_3d(:,:,1); p1 = p1(:);
p2 = real_3d(:,:,2); p2 = p2(:);
p3 = real_3d(:,:,3); p3 = p3(:);

r = left_image_rgb(:,:,1); r = r(:);
g = left_image_rgb(:,:,2); g = g(:);
b = left_image_rgb(:,:,3); b = b(:);

col = double([r g b])/255;

% data subsampling
idx = 1:2:20000;
p1 = p1(idx);
p2 = p2(idx);
p3 = p3(idx);

col = col(idx,:);
figure(hvs_handle)
scatter3(p3,-p1,-p2,5,col);




                                                                                                                                                                                                                                                                                                                                                                                                                                        


axis equal