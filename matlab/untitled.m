function [point_1,point_2]=line_line_closest_point(x_origin, u,v)

x_origin=0.178261;
w_0=[-x_origin,0,0];

l_origin=[0,0,0];
r_origin=[x_origin,0,0];


a=dot(u,u);
b=dot(u,v);
c=dot(v,v);
d=dot(u,w_0);
e=dot(v,w_0);

denominator=(a*c)-(b*b);

numerator_1=(b*e)-(c*d);
numerator_2=(a*e)-(b*d);
sc=numerator_1/denominator;
tc=numerator_2/denominator;


point_1=sc*u;
point_2=[x_origin,0,0]'+tc*v;



% % Their vertial concatenation is what you want
pts = [l_origin; point_1'; ...
    r_origin; point_2'];

% % Alternatively, you could use plot3:
plot3(pts(1:2,1), pts(1:2,2), pts(1:2,3),'r')
hold on
plot3(pts(3:4,1), pts(3:4,2), pts(3:4,3),'b')
xlabel('x'); ylabel('y');