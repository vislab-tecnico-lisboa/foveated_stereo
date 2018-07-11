function logpolar_image=ReticalCortical(figure_handle, eye_transformation, image, image_retina, logpolar_image)
n_angles=128;
eye_retina_radii=32;
eye_iris_radii=20;
eye_radius=0.008;
eye_center_offset=1.0;
[x_old,y_old,z_old] = sphere(n_angles-1);

ones_=ones(size(x_old));
rotate_eye_180=[-1.0000         0   -0.0000;
         0    1.0000         0;
    0.0000         0   -1.0000];
points=eye_transformation(1:3,1:3)'*rotate_eye_180*[eye_radius*x_old(:),eye_radius*y_old(:),eye_radius*(z_old(:)+eye_center_offset)]';

x=reshape(points(1,:)-eye_transformation(1,4), size(x_old));
y=reshape(points(2,:)-eye_transformation(2,4), size(y_old));
z=reshape(points(3,:)-eye_transformation(3,4), size(z_old));
%% DRAW THE EYE (polar coordinates)

cart_eye = imread('eye.png');
polar_eye=uint8(imgpolarcoord(cart_eye,size(cart_eye,2)/2-1,360));
%polar_eye=permute(polar_eye,[2 1 3]);
% eye_mesh=surf(z,x,y,'CData',polar_eye,'FaceColor','texturemap');

eye_mesh=surf2patch(surf(z(1:eye_iris_radii,1:n_angles),...
              -x(1:eye_iris_radii,1:n_angles),...
              -y(1:eye_iris_radii,1:n_angles),'EdgeColor','none','CData',polar_eye,'FaceColor','texturemap'));

%set(eye_mesh,'EdgeColor','none')
%set(gca,'xticklabel',[])
%set(gca,'xtick',[])
%set(gca,'yticklabel',[])
%set(gca,'ytick',[])
%set(gca,'zticklabel',[])
%set(gca,'ztick',[])
%text(0.5,0.0,1.1,'Retina')
%text(3.5,0.0,0.5,'Cortex')

hold on

%% DRAW THE OCULAR GLOBE (semi-sphere)
% visualization_offset=60;
% ocular_globe_mesh=mesh(z(eye_iris_radii:end-eye_retina_radii,1:n_angles),...
%                        -x(eye_iris_radii:end-eye_retina_radii,1:n_angles),...
%                        -y(eye_iris_radii:end-eye_retina_radii,1:n_angles));
% set(ocular_globe_mesh,'facecolor','w','EdgeColor','none')

%% DRAW THE RETINA (logpolar)
% logpolar ego-sphere angles
rmin=log(0.01);
rmax=log(max(max(abs(x_old(1:eye_retina_radii,1:n_angles)))));

%RMAX = xc;
%xc=max(max(x(1:eye_retina_radii,1:n_angles)));

% Eye receptive fields log polar distribution
rho = linspace(rmin,rmax, eye_retina_radii);
eta = linspace(0,2*pi,n_angles);

% planar coordinates in the retina
x_ = eye_radius*exp(rho')*cos(eta);
y_ = eye_radius*exp(rho')*sin(eta);
%r=sqrt(x_.^2+y_.^2+z(1:eye_retina_radii,1:n_angles).^2);
r_=eye_radius*1.0;
z_=sqrt(-x_.^2+-y_.^2+r_.^2);
theta=atan2(y_,x_);
phi=acos(z_(1:eye_retina_radii,1:n_angles)./r_);

% convert to cartesian azimuth
Xretina_old=r_.*cos(theta).*sin(phi);
Yretina_old=r_.*sin(theta).*sin(phi);
Zretina_old=r_.*cos(phi);

points=eye_transformation(1:3,1:3)'*rotate_eye_180*[Xretina_old(:),Yretina_old(:),Zretina_old(:)+eye_radius]';
Xretina=reshape(points(1,:)-eye_transformation(1,4),size(Xretina_old));
Yretina=reshape(points(2,:)-eye_transformation(2,4),size(Yretina_old));
Zretina=reshape(points(3,:)-eye_transformation(3,4),size(Zretina_old));

% Xretina=Xretina_old;
% Yretina=Yretina_old;
% Zretina=Zretina_old;

%logpolar_image = to_cortical(foveal,image');
%subplot(figure_handle);
figure(figure_handle);
retinal_mesh=surf2patch(surf(Zretina,-Xretina,-Yretina,'EdgeColor','none','CData',double(logpolar_image)','FaceColor','texturemap'));
%retinal_mesh=surf2patch(surf(Zretina,-Xretina,-Yretina,'EdgeColor','none','CData',double(logpolar_image)','FaceColor','w'));
%alpha(0.5)
cmap = contrast(double(logpolar_image)'); colormap(cmap);
%set(retinal_mesh,'EdgeColor','none');
return

%% DRAW CORTICAL
subplot(figure_handle)

[corticalX,corticalY]=meshgrid(-size(logpolar_image,2)/2:size(logpolar_image,2)/2,...
                               -size(logpolar_image,1)/2:size(logpolar_image,1)/2);
corticalX=corticalX/size(logpolar_image,2);
corticalY=corticalY/size(logpolar_image,1);
corticalZ=3*ones(size(corticalY));

cortical_mesh=surf2patch(surf(corticalZ,-corticalX,-corticalY,'EdgeColor','none','CData',double(logpolar_image)','FaceColor','texturemap'));
cmap = contrast(double(logpolar_image)'); colormap(cmap);


%% Draw image
% imageSize = size(image); 
% ci = [size(image,2)/2, size(image,1)/2, size(image,2)/2];     % center and radius of circle ([c_row, c_col, r])
% [xx,yy] = meshgrid((1:imageSize(2))-ci(2),(1:imageSize(1))-ci(1));
% image((xx.^2 + yy.^2)>ci(3)^2)=255;
% 
% xx=2*xx/imageSize(2);
% yy=2*yy/imageSize(2);
% Zimage=zeros(size(image));
% ci(3)
% image_mesh=surf(Zimage-3,-yy,-xx,'CData',double(image)','FaceColor','texturemap');
% set(image_mesh,'EdgeColor','none')

%axis equal;  daspect([1 1 1]);
%set(gca,'Visible','off')

