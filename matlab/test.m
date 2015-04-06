clc
close all
clear all
I = imread('cameraman.tif');

M=size(I,1);
N=size(I,2);
RMIN=1;
RMAX=min(M,N);
NRINGS=200;
NSECTORS=360;
xc=N/2;
yc=M/2;

A=logsample(I, RMIN, RMAX, xc, yc, NRINGS, NSECTORS);
B=logsampback(A, RMIN, RMAX);
figure
subplot(1,3,1)
imshow(I)

subplot(1,3,2)
imshow(A)

subplot(1,3,3)
imshow(B)
