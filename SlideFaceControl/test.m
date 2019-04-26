clear,clc,close all

% 快门时间72.6us-4.7s
% 分辨率2592*1944
% 焦距4-35mm
% 视场角H 82-8，V66-7


colorImage = imread('handicapSign.jpg');
I = rgb2gray(colorImage);

% Detect MSER regions.
[mserRegions] = detectMSERFeatures(I, ...
    'RegionAreaRange',[200 8000],'ThresholdDelta',4);

figure
imshow(I)
hold on
plot(mserRegions, 'showPixelList', true,'showEllipses',false)
title('MSER regions')
hold off