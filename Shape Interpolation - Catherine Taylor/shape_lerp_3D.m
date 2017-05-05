% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out 3D shape deformation using linear interpolation.

close all;
clear;

obj1 = readObj('cube1.obj'); %reads object file and stores vertices and faces.
obj2 = readObj('cube2.obj');
obj3 = readObj('cube3.obj');
FV1 = obj1.f.v;
V1 = obj1.v;
V2 = obj2.v;
V3 = obj3.v;

figure
interpolations =100;

for i=1:interpolations+1 %carries out linear interpolation from obj 1 to 2.
    t=1/interpolations*(i-1);
    V_new = (1-t)*V1 + t*V2;
    trimesh(FV1(:,1:3), V_new(:,1), V_new(:,2), V_new(:,3));
    title('3D Linear Interpolation')
    axis([-2, 2, 0, 4,-2,2]);
    drawnow;
end

for i=1:interpolations+1 %carries out linear interpolation from obj 1 to 2.
    t=1/interpolations*(i-1);
    V_new = (1-t)*V2 + t*V3;
    trimesh(FV1(:,1:3), V_new(:,1), V_new(:,2), V_new(:,3));
    title('3D Linear Interpolation')
    axis([-2, 2, 0, 4,-2,2]);
    drawnow;
end
