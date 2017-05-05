% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out 2D shape deformation using linear interpolation.

close all;
clear;

obj1 = readObj('dino.obj'); %reads object file and stores vertices and faces.
obj2 = readObj('keyframe1.obj');
obj3 = readObj('keyframe2.obj');
obj4 = readObj('keyframe3.obj');
obj5 = readObj('keyframe4.obj');
obj6 = readObj('keyframe5.obj');

FV1 = obj1.f.v;
V1 = obj1.v;
V2 = obj2.v;
V3 = obj3.v;
V4 = obj4.v;
V5 = obj5.v;
V6 = obj6.v;

figure('units','normalized','outerposition',[0 0 1 1]);
title('Linear Interpolation Animation')
interpolations = 50;

for i=1:interpolations %carries out linear interpolation from obj 1 to 2.
    V_new = (1-1/interpolations*(i-1))*V1 + 1/interpolations*(i-1)*V2;
    trimesh(FV1, V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    title('Linear Interpolation Animation')
    drawnow;
end

for i=1:interpolations %carries out linear interpolation from obj 2 to 3.
    V_new = (1-1/interpolations*(i-1))*V2 + 1/interpolations*(i-1)*V3;
    trimesh(FV1, V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    title('Linear Interpolation Animation')
    drawnow;
end
for i=1:interpolations %carries out linear interpolation from obj 3 to 4.
    V_new = (1-1/interpolations*(i-1))*V3 + 1/interpolations*(i-1)*V4;
    trimesh(FV1, V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    title('Linear Interpolation Animation')
    drawnow;
end
for i=1:interpolations %carries out linear interpolation from obj 4 to 5.
    V_new = (1-1/interpolations*(i-1))*V4 + 1/interpolations*(i-1)*V5;
    trimesh(FV1, V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    title('Linear Interpolation Animation')
    drawnow;
end
for i=1:interpolations%carries out linear interpolation from obj 5 to 6.
    V_new = (1-1/interpolations*(i-1))*V5 + 1/interpolations*(i-1)*V6;
    trimesh(FV1, V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    title('Linear Interpolation Animation')
    drawnow;
end