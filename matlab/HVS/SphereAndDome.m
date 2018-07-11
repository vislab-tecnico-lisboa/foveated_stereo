close all
n_angles=16;
f=6;
faceopaque=1;
quiver=0;
r=1;
make_plot=1;
figure
[x,y,z] = sphere(n_angles);
sphere_mesh=mesh(x,...
    y,...
    z)%'CData',polar_eye,'FaceColor','texturemap');

set(sphere_mesh,'EdgeColor','k');
if faceopaque == 0
    set(sphere_mesh,'FaceColor','none')
end

if quiver == 1
    hold on
    quiver3(zeros(3,1),zeros(3,1),zeros(3,1),[0;0;2],[0;2;0],-[2;0;0])
end

az = 125.0;
el = 25;
view(az, el);
axis equal;  daspect([1 1 1]);
set(gca,'Visible','off')
set(gcf, 'Color', [1,1,1]);

[ x, y, z, TRI]=make_icosahedron(f, r, make_plot , faceopaque);
figure
if faceopaque == 1
    h=trisurf(TRI,x,y,z, 'FaceColor', [1 1 1], 'EdgeColor', 0*[1 1 1], 'LineWidth', 1 );
else
    h=trisurf(TRI,x,y,z, 'FaceColor', 'none', 'EdgeColor', 0*[1 1 1],'LineWidth', 1 );
end
axis equal;

xlabel('x-axis (m)', 'Fontsize', 16');
ylabel('y-axis (m)', 'Fontsize', 16');
zlabel('z-axis (m)', 'Fontsize', 16');
title( ['Microphone Structure Struts and Nodes'], 'Fontsize', 16');

if quiver == 1
    hold on
    quiver3(zeros(3,1),zeros(3,1),zeros(3,1),[0;0;2],[0;2;0],-[2;0;0])
end
az = 125.0;
el = 25;
view(az, el);
axis equal;  daspect([1 1 1]);
set(gca,'Visible','off')
axis equal
set(gcf, 'Color', [1,1,1]);

%and egosphere
%% Graphic
f = figure;
clf(f);
set(f,'Renderer','zbuffer');
ax = axes('Parent', f);
hold(ax, 'on');
axis(ax,'equal');
n = 1000;

mu = [0.0,0.0,0.0];
sigma = [2.0,0.0,0.0;...
         0.0,2.0,0.0;...
         0.0,0.0,0.05];
rng default  % For reproducibility
xyz = mvnrnd(mu,sigma,n)';
xyz = bsxfun(@rdivide, xyz, sqrt(sum(xyz.^2,1)));
[P, K, voronoiboundary] = voronoisphere(xyz);



plot3(ax, xyz(1,:),xyz(2,:),xyz(3,:),'k.');
clmap = cool();
ncl = size(clmap,1);
for k = 1:n
    X = voronoiboundary{k};
    cl = clmap(mod(k,ncl)+1,:);
    fill3(X(1,:),X(2,:),X(3,:),[1 1 1],'Parent',ax,'EdgeColor','k');
    %alpha(.3)
end
%alpha(.3)
set(ax,'Visible','off')
axis equal
set(gcf, 'Color', [1,1,1]);
axis equal;  daspect([1 1 1]);
set(gca,'Visible','off')
axis equal
set(gcf, 'Color', [1,1,1]);





% [P, K, voronoiboundary] = voronoisphere(xyz);
% f = figure;
% clf(f);
% set(f,'Renderer','zbuffer');
% ax = axes('Parent', f);
% hold(ax, 'on');
% 
% cla(ax);
% plot3(ax, P(3,:),-P(1,:),-P(2,:), 'b.','markersize',5.0);
% 
% [x,y,z] = sphere(n_angles);
% sphere_mesh=mesh(x,...
%     y,...
%     z,'EdgeColor','k')
% set(gca,'Visible','off')
% axis equal
% set(0,'defaultaxesposition',[0 0 1 1]) 
% %'CData',polar_eye,'FaceColor','texturemap');
% % for k = 1:n
% %     V = P(:,K{k});
% %     V = V(:,[1:end 1]);
% %     for i=1:length(V)-1
% %         plot3(ax, V(1,i:i+1),V(2,i:i+1),V(3,i:i+1), 'r', 'Linewidth', 1);
% %     end
% % end
% axis equal;  daspect([1 1 1]);
% set(gca,'Visible','off')
% set(gcf, 'Color', [1,1,1]);
% %set(f, 'Position', get(0,'Screensize')); % Maximize figure. 
% 
% az = 125.0;
% el = 25;
% view(az, el);
% 
% %export_fig -pdf random_ego_sphere;








% %% DRAW EYES
% 
% n_angles=128;
% eye_retina_radii=32;
% eye_iris_radii=20;
% eye_radius=0.008;
% eye_center_offset=1.0;
% [x_old,y_old,z_old] = sphere(n_angles-1);
% 
% ones_=ones(size(x_old));
% rotate_eye_180=[-1.0000         0   -0.0000;
%          0    1.0000         0;
%     0.0000         0   -1.0000];
% points=eye_transformation(1:3,1:3)'*rotate_eye_180*[eye_radius*x_old(:),eye_radius*y_old(:),eye_radius*(z_old(:)+eye_center_offset)]';
% 
% x=reshape(points(1,:)-eye_transformation(1,4), size(x_old));
% y=reshape(points(2,:)-eye_transformation(2,4), size(y_old));
% z=reshape(points(3,:)-eye_transformation(3,4), size(z_old));
% %% DRAW THE EYE (polar coordinates)
% 
% cart_eye = imread('eye.png');
% polar_eye=uint8(imgpolarcoord(cart_eye,size(cart_eye,2)/2-1,360));
% %polar_eye=permute(polar_eye,[2 1 3]);
% % eye_mesh=surf(z,x,y,'CData',polar_eye,'FaceColor','texturemap');
% 
% eye_mesh=surf2patch(surf(z(1:eye_iris_radii,1:n_angles),...
%               -x(1:eye_iris_radii,1:n_angles),...
%               -y(1:eye_iris_radii,1:n_angles),'EdgeColor','none','CData',polar_eye,'FaceColor','texturemap'));
