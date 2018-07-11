close all

%scaling_factor=0.01;
% images = dir('Adirondack-perfect/*.png');
% left_image_rgb  = imread('Adirondack-perfect/im0.png');
% left_image_gray  = rgb2gray(left_image_rgb);

%CROP IMAGES (SHOULD BE SQUARED)
% if size(left_image_rgb,1)~=size(left_image_rgb,2)
%     c = .5*size(left_image_gray); % center
%     w = min( size(left_image_gray) ); % take min dimension
%     bottom=ceil([c(2) c(1)]-.5*[w w])+1;
%     rect = [ bottom, [w-1 w-1] ];
%     left_image_gray = imcrop( left_image_gray, rect);
%     left_image_rgb = imcrop( left_image_rgb, rect);
%
% end
%
% % CROP AGAIN TO GET CIRCULAR
% ci = [size(left_image_gray,2)/2, size(left_image_gray,2)/2, size(left_image_gray,2)/2];     % center and radius of circle ([c_row, c_col, r])
% [xx,yy] = ndgrid((1:size(left_image_gray,2))-ci(1),(1:size(left_image_gray,2))-ci(2));
% mask = uint8((xx.^2 + yy.^2)<ci(3)^2);
% croppedImage = uint8(zeros(size(left_image_rgb)));
% croppedImage(:,:,1) = left_image_rgb(:,:,1).*mask;
% croppedImage(:,:,2) = left_image_rgb(:,:,2).*mask;
% croppedImage(:,:,3) = left_image_rgb(:,:,3).*mask;
% left_image_rgb=croppedImage;
%imshow(croppedImage);

%%%%%%%%%%%%%%%%%%%
% CARTESIAN IMAGE %
%%%%%%%%%%%%%%%%%%%
M=15;
N=15;

jc=N/2;
ic=M/2;

pmax=0.5*min(M,N);
po=2.0;
fovea_pixels_m=5;
fovea_pixels_n=5;

%MEANS
std_dev_plot=3.0;

L=4.0;
alpha=0.25;
ki=20.0;
beta=2.0;
lambda=(alpha^2)*(L+ki)-L;

plots_figure=figure('units','normalized','outerposition',[0 0 1 1])
%subplot(1,2,1);
%title('Uniformly Distributed RFs')

means=zeros(N*M,2);
covs=zeros(N*M,2,2);
sigma_points=zeros(N*M,2,5);
total_pixels_cart=0
total_std_dev_inv_conv=0
std_dev=(1/(2*3.0));
for r=0:M-1
    for c=0:N-1
        if sqrt((c+0.5-jc)^2+(r+0.5-ic)^2)>M/2
            continue
        end
        total_pixels_cart=total_pixels_cart+1;
        index=c+1+(r)*N;
        means(index,:)=[r+0.5 c+0.5];
        covs(index,:,:)=[std_dev^2 0 ; 0 std_dev^2];
        total_std_dev_inv_conv=total_std_dev_inv_conv+std_dev^2;
        sigma_points(index,:,:)=sigmas(means(index,:)', reshape(covs(index,:,:),2,2), lambda+L);
        plot_gaussian_ellipsoid(means(index,:)', reshape(covs(index,:,:),2,2), std_dev_plot);
        %plot(reshape(sigma_points(index,1,:),1,5),reshape(sigma_points(index,2,:),1,5)','xr')
    end
end

%plot(ic,jc,'rx')

% DRAW GRID
grid_rectangular ( 0, N, N+1, 0, M, M+1 )
% DRAW ROI
circles(ic,jc,pmax,'facecolor','none')
axis equal
daspect([1 1 1]);
%axis off
set(gcf,'color','w');
set(gca,'Visible','off')
axis([-1,N+2,-1,M+2])
% string='conventional_rfs';
% export_fig('-pdf','-r600',string)

%%%%%%%%%%%%%%%%%%%
% LOG-POLAR IMAGE %
%%%%%%%%%%%%%%%%%%%

%S=M;%S=round(2*pi/(a-1));
syms R

eq=pmax^2*pi/R==2*pi/(exp(log(pmax/po)/R)-1);
R=abs(round(double(solve(eq,R))));
S=abs(round(pmax^2*pi/R));
a=exp(log(pmax/po)/R);

% R=round(log(pmax/po)*S/(2*pi));
% S=round(S);
std_devs=zeros(R,1);
for i=0:R-1
    std_devs(i+1)=po*(a-1)*a^(i-1)/3;
end
q=S/(2*pi);
means_retinal=zeros(S*R,2);
covs_retinal=zeros(S*R,2,2);
sigma_points_retinal=zeros(S*R,2,5);


plots_figure2=figure('units','normalized','outerposition',[0 0 1 1])
%subplot(1,2,2);
title('Log-polar Distributed RFs')
total_pixels_log=0;

total_std_dev_inv_retinal=0
for s=0:S-1
    for r=0:R-1
        index=r+1+s*R;
        total_pixels_log=total_pixels_log+1;
        % computing corresponding cartesian coordinates
        x=(po*(a^r)*cos(s/q)+ic);
        y=(po*(a^r)*sin(s/q)+jc);
        means_retinal(index,:)=[x y];
        %std_dev=(1/(2*3.0))*(sqrt((x-ic)^2+(y-jc)^2));

        hold on;

        covs_retinal(index,:,:)=[std_devs(r+1)^2 0 ; 0 std_devs(r+1)^2];
        total_std_dev_inv_retinal=total_std_dev_inv_retinal+std_devs(r+1)^2;
        circles_retinal(index,:)=[x y std_devs(r+1)];
        plot_gaussian_ellipsoid(means_retinal(index,:)', reshape(covs_retinal(index,:,:),2,2), std_dev_plot);
        sigma_points_retinal(index,:,:)=sigmas(means_retinal(index,:)', reshape(covs_retinal(index,:,:),2,2), lambda+L);
        %plot(reshape(sigma_points_retinal(index,1,:),1,5),reshape(sigma_points_retinal(index,2,:),1,5)','xr')

        %plot sigma points
        %plot(reshape(sigma_points_retinal(index,1,1),1,1),reshape(sigma_points_retinal(index,2,1),1,1)','rx') % mean sigma point
        %plot(reshape(sigma_points_retinal(index,1,2:end),1,4),reshape(sigma_points_retinal(index,2,2:end),1,4)','rx') % others
    end
end

means_foveal=zeros(fovea_pixels_n*fovea_pixels_m,2);
covs_foveal=zeros(fovea_pixels_n*fovea_pixels_m,2,2);

%std_dev_test=(1/(3*2))*(0.5*pmax/fovea_pixels_n);
for r=0:fovea_pixels_m-1
    for c=0:fovea_pixels_n-1
        if sqrt(((r+0.5-fovea_pixels_m/2)/fovea_pixels_m)^2+((c+0.5-fovea_pixels_n/2)/fovea_pixels_n)^2)>po
            continue
        end
        index=c+1+(r)*N;
        means_foveal(index,:)=[2*po*(r+0.5)/fovea_pixels_m+ic-po 2*po*(c+0.5)/fovea_pixels_n+jc-po];
        %std_dev=(1/(2*3.0))*(2*po)/fovea_pixels_m;
        covs_foveal(index,:,:)=[std_devs(1)^2 0 ; 0 std_devs(1)^2];
        sigma_points(index,:,:)=sigmas(means_foveal(index,:)', reshape(covs_foveal(index,:,:),2,2), lambda+L);
        plot_gaussian_ellipsoid(means_foveal(index,:)', reshape(covs_foveal(index,:,:),2,2), std_dev_plot);
    end
end

%plot(ic,jc,'rx')
% DRAW GRID
grid_rectangular ( 0, N, N+1, 0, M, M+1 )
% DRAW ROI
circles(ic,jc,pmax,'facecolor','none')
axis equal
daspect([1 1 1]);
%axis off
set(gcf,'color','w');
set(gca,'Visible','off')
axis([-1,N+2,-1,M+2])
string='log_polar_rfs';
%export_fig('-pdf','-r600',string)


