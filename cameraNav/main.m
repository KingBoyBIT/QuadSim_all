clear,clc,close all
if isempty(imaqfind)~=1
	delete(imaqfind)% 关闭正在占用的摄像头
end
imaqhwinfo;

obj1 = videoinput('winvideo',1,'YUY2_640x480');
set(obj1,'ReturnedColorSpace','rgb');
triggerconfig(obj1,'manual');  
fig1=figure(1);
start(obj1);
%%
snapshot1 = getsnapshot(obj1);
hsv = rgb2hsv(snapshot1);
V = hsv(:,:,3); % 亮度
K = 0.1;
hsv(:,:,3) = V *K;
rgb1 = hsv2rgb(hsv);
rgb1=rgb1+(1-rgb1).*rgb1;
imshow(rgb1(:,:,3))