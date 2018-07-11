clc
clear all
close all

addpath(genpath('.'))

images = dir('Adirondack-perfect/*.png');
left_image_rgb  = imread('Adirondack-perfect/im0.png');
right_image_rgb = imread('Adirondack-perfect/im1.png');
disparity_map = imread('Adirondack-perfect/disp0-n.pgm');
%f=fopen('Adirondack-perfect/disp0.pfm');
%disparity_map = fread(f);

f=fopen('Adirondack-perfect/calib.txt');
l=fgetl(f);
left_cam_params =cameraParameters('IntrinsicMatrix',cell2mat(reshape(textscan(l,'cam0=[%f %f %f; %f %f %f; %f %f %f]'),3,3)));
l=fgetl(f);
right_cam_params=cameraParameters('IntrinsicMatrix',cell2mat(reshape(textscan(l,'cam1=[%f %f %f; %f %f %f; %f %f %f]'),3,3)));

f=left_cam_params.IntrinsicMatrix(1,1)
baseline=176.252;
doffs=209.059
translationOfCamera2=[10 0 0];
rotationOfCamera2=[1 0 0; 0 1 0; 0 0 1];
%[J1_full,J2_full] = rectifyStereoImages(left_image_rgb,right_image_rgb, stereoParams,'OutputView','full')
%pointCloud = reconstructScene(single(disparity_map),stereoParams)
Z = baseline * f ./ (double(disparity_map) + double(doffs));



% calibrate images
left_image_rgb   = undistortImage(left_image_rgb,left_cam_params);
right_image_rgb  = undistortImage(right_image_rgb,right_cam_params);

left_image_gray  = rgb2gray(left_image_rgb);
right_image_gray = rgb2gray(right_image_rgb);


%CROP IMAGES (SHOULD BE SQUARED)
if size(left_image_rgb,1)~=size(left_image_rgb,2)
    c = .5*size(left_image_gray); % center
    w = min( size(left_image_gray) ); % take min dimension
    bottom=ceil([c(2) c(1)]-.5*[w w])+1;
    rect = [ bottom, [w-1 w-1] ];
    left_image_gray = imcrop( left_image_gray, rect);
    right_image_gray = imcrop(right_image_gray, rect);
    disparity_map_gray = imcrop(disparity_map, rect);
end
figure
subplot(2,1,1)
imshow(left_image_gray);
subplot(2,1,2)
imshow(right_image_gray);

%%%%%%%%%%%%%%%%%%%%%%%
% Get cortical images %
%%%%%%%%%%%%%%%%%%%%%%%

min_disparity=-48;
max_disparity=48;
horizontal_disparity_step=1;
horizontal_disparities=min_disparity:horizontal_disparity_step:max_disparity-1;

horizontal_total_disparities=size(horizontal_disparities,2);

sigma=3.0;
q=0.1;
pole=0.8;
focal_distance=1.0;
spherical_angle_bins=360;
M=size(left_image_gray,1);
N=size(left_image_gray,2);
NRINGS=256;
NSECTORS=256;
RMIN=1;
RMAX=min(M,N);
xc=N/2;
yc=M/2;
interp=1;
sp=0;
full=0;
uncertainty_lower_bound=1.0;
L=4.0;
alpha=0.25;
ki=3.0;
beta=2.0;
scaling_factor=0.001;
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
foveal = foveal_interface(N,...
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
           % Create a class instance

disp('transform images to the cortical domain');
tic
left_image_log_gray =to_cortical(foveal,left_image_gray');
right_image_log_gray=to_cortical(foveal,right_image_gray');
toc

%% get cortical warp tables
% disp('get cortical warp maps');
% tic
% [eta, rho]=get_delta_logpolar_warp_maps(foveal);
% toc

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
%     bigsubplot(1,3,1,3,0); imshow(cartesian_warped);
%     drawnow;
% end

% disp('computing disparity maps');
% tic
% disparity_map=compute_disparity_map(foveal, left_image_gray', right_image_gray',length(horizontal_disparities),min_disparity);
% toc
% 
% disparity_map=uint8(disparity_map);
% cortical_disparity_map=to_cortical(foveal, disparity_map');

%%%%%%%%%%%%%%%%%%%
% CORTICAL STEREO %
%%%%%%%%%%%%%%%%%%%

figure_handle=subplot(1,1,1);
RetinalCortical(foveal, left_image_gray,  left_image_log_gray,  figure_handle, -1.5);
RetinalCortical(foveal, right_image_gray, right_image_log_gray, figure_handle,  1.5);

axis equal;  daspect([1 1 1]);
set(gca,'Visible','off')


%% Compute stereo
%points_3d=get_3d_points(foveal);
pointCloud = reconstructScene(disparity_map_gray,stereoParams)

%% Draw cortical disparity image
% imageSize = size(cortical_disparity_map);
% ci = [size(cortical_disparity_map,2)/2, size(cortical_disparity_map,1)/2, size(cortical_disparity_map,2)/2];     % center and radius of circle ([c_row, c_col, r])
% [xx,yy] = meshgrid((1:imageSize(2))-ci(2),(1:imageSize(1))-ci(1));
% %cortical_disparity_map((xx.^2 + yy.^2)>ci(3)^2)=255;
% 
% xx=2*xx/imageSize(2);
% yy=2*yy/imageSize(2);
% Zimage=zeros(size(cortical_disparity_map));
% 
% image_mesh=surf(Zimage+5,-yy,-xx,'CData',double(cortical_disparity_map)','FaceColor','texturemap');
% set(image_mesh,'EdgeColor','none')
% 
% 
% axis equal;  daspect([1 1 1]);
% set(gca,'Visible','off')


%% draw egosphere
% egosphere=get_egosphere(foveal);
% ego_topology=[];
% ego_planar_topology=[];
% 
% for i=1:length(egosphere)
%     ego_planar_topology=[ego_planar_topology [egosphere(i).spherical_topology.phi egosphere(i).spherical_topology.theta]' ];
%     ego_topology=[ego_topology cell2mat(struct2cell(egosphere(i).cartesian_topology))];
% end
% figure(10)
% voronoi(ego_planar_topology(1,:),ego_planar_topology(2,:));
% xyz=ego_topology;
% [P, K, voronoiboundary] = voronoisphere(xyz);

%% Graphic
% f = figure(3);
% clf(f);
% set(f,'Renderer','zbuffer');
% ax = axes('Parent', f);
% hold(ax, 'on');
% axis(ax,'equal');
% 
% plot3(ax, xyz(1,:),xyz(2,:),xyz(3,:),'w.');
% clmap = cool();
% ncl = size(clmap,1);
% for k = 1:length(xyz)
%     X = voronoiboundary{k};
%     cl = clmap(mod(k,ncl)+1,:);
%     fill3(X(1,:),X(2,:),X(3,:),cl,'Parent',ax,'EdgeColor','w');
% end
% axis(ax,'equal');
% axis(ax,[-1 1 -1 1 -1 1]);
% return
% 
% % Pretend function TriDip exists that takes X,Y,Z and Tri(angulation) info &
% % returns a column vector (same length as Tri) with angle of dip for each triangle
% % patch defined by each row of Tri (i.e., vertex indices of single triangle patch)
% C_Tri = TriDip(xyz(1,:), xyz(2,:), xyz(3,:),tri);
% hh = trisurf(tri,X,Y,Z);
% 
% % Additional bit to control color of each patch individually
% set(gca,'CLim',[min(C_Tri), max(C_Tri)]);
% set(hh,'FaceColor','flat',...
%     'FaceVertexCData',C_Tri,...
%     'CDataMapping','scaled');
% 
% return
% 
% subplot(figure_handle);
% 
% retinal_mesh=surf(xyz(1,:),xyz(2,:),xyz(3,:), 'CData', left_image_log_gray', 'FaceColor', 'texturemap');
% cmap = contrast(double(logpolar_image)'); colormap(cmap);
% set(retinal_mesh,'EdgeColor','none');
% 
% return;
% 
% figure(1)
% subplot(1,3,1)
% imshow(left_image_gray)
% 
% subplot(1,3,2)
% imshow(left_image_log_gray)
% 
% retinal=to_cartesian(foveal,left_image_log_gray');
% 
% subplot(1,3,3)
% imshow(retinal)
% 
% figure(2)
% for i=2:2:2*horizontal_total_disparities
%     subplot(horizontal_total_disparities,2,i-1);
%     point_density=300;
%     [X Y]=meshgrid(1:point_density:N,1:point_density:M);
%     quiver(X,...
%         Y,...
%         cartesian_disparity_maps(1:point_density:M,1:point_density:N,2,i/2),...
%         cartesian_disparity_maps(1:point_density:M,1:point_density:N,1,i/2));
%     axis([0,N , 0,M])
%     subplot(horizontal_total_disparities,2,i);
%     point_density=1;
%     [P THETA]=meshgrid(1:point_density:NRINGS,1:point_density:NSECTORS);
%     
%     quiver(P,...
%         THETA,...
%         rho(1:point_density:NSECTORS,1:point_density:NRINGS,i/2),...
%         eta(1:point_density:NSECTORS,1:point_density:NRINGS,i/2));
%     
%     axis([0,NRINGS,0,NSECTORS])
% end
% 
