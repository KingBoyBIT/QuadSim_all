clear,clc,close all
if isempty(imaqfind)~=1
	delete(imaqfind)% �ر�����ռ�õ�����ͷ
end
imaqhwinfo;

obj1 = videoinput('winvideo',1,'YUY2_640x480');
set(obj1,'ReturnedColorSpace','rgb');
triggerconfig(obj1,'manual');  
fig1=figure(1);
start(obj1);
%%
snapshot1 = getsnapshot(obj1);
imshow(snapshot1(:,:,3))