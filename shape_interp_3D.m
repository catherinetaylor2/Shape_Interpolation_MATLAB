% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out 3D shape deformation using linear interpolation.

close all;
clear;

obj1 = readObj('cube1.obj'); %reads object file and stores vertices and faces.
FV1 = obj1.f.v;
V1 = obj1.v;

obj2 = readObj('cube2.obj'); %reads object file and stores vertices and faces.
FV2 = obj2.f.v;
V2 = obj2.v;

% figure
% trimesh(FV1(:,1:3), V1(:,1), V1(:,2), V1(:,3)); 
% axis([0, 4, 0, 4,0,4]);

for i=1:100 %carries out linear interpolation from obj 1 to 2.
    V_new = (1-1/100*(i-1))*V1 + 1/100*(i-1)*V2;
    trimesh(FV1(:,1:3), V_new(:,1), V_new(:,2), V_new(:,3));
    axis([0, 4, 0, 4,-4,4]);
    drawnow;
end
