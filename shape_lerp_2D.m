% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out 2D shape deformation using linear interpolation.

close all;
clear;

obj1 = readObj('dino.obj'); %reads object file and stores vertices and faces.
FV1 = obj1.f.v;
V1 = obj1.v;

obj2 = readObj('keyframe1.obj');
FV2 = obj2.f.v;
V2 = obj2.v;

obj3 = readObj('keyframe2.obj');
FV3 = obj3.f.v;
V3 = obj3.v;

obj4 = readObj('keyframe3.obj');
FV4 = obj4.f.v;
V4 = obj4.v;

obj5 = readObj('keyframe4.obj');
FV5 = obj5.f.v;
V5 = obj5.v;

obj6 = readObj('keyframe5.obj');
FV6 = obj6.f.v;
V6 = obj6.v;

figure
trimesh(FV1(:,1:3), V1(:,1), V1(:,2)); 
axis([-18 15 -15 15])

interpolations = 100;

for i=1:interpolations %carries out linear interpolation from obj 1 to 2.
    V_new = (1-1/interpolations*(i-1))*V1 + 1/interpolations*(i-1)*V2;
    if (1/interpolations*(i-1)==0.5)
       y=9; 
    end
    trimesh(FV1(:,1:3), V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    drawnow;
end

for i=1:interpolations %carries out linear interpolation from obj 2 to 3.
    V_new = (1-1/interpolations*(i-1))*V2 + 1/interpolations*(i-1)*V3;
    trimesh(FV1(:,1:3), V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    drawnow;
end
for i=1:interpolations %carries out linear interpolation from obj 3 to 4.
    V_new = (1-1/interpolations*(i-1))*V3 + 1/interpolations*(i-1)*V4;
    trimesh(FV1(:,1:3), V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    drawnow;
end
for i=1:interpolations %carries out linear interpolation from obj 4 to 5.
    V_new = (1-1/interpolations*(i-1))*V4 + 1/interpolations*(i-1)*V5;
    trimesh(FV1(:,1:3), V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    drawnow;
end
for i=1:100 %carries out linear interpolation from obj 5 to 6.
    V_new = (1-1/interpolations*(i-1))*V5 + 1/interpolations*(i-1)*V6;
    trimesh(FV1(:,1:3), V_new(:,1), V_new(:,2));
    axis([-20 20 -15 15])
    drawnow;
end