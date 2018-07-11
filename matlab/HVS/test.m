clc
close all
clear all



%% Create stereoscopic vision system
% resolution
M=100;
N=100;
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
    full);

img=imread('peppers.png');

lena_logpolar = to_cortical(foveal,img); 
lena_cart_back =  to_cartesian(foveal,lena_logpolar);
himg = imshow(lena_cart_back);
for i=1:30
    for j=1:30
        lena_logpolar=to_cortical(foveal,img); 
        lena_cart_back = to_cartesian(foveal,lena_logpolar);
        set(himg, 'CData', lena_cart_back);  %instead of imshow
        drawnow
    end
end








