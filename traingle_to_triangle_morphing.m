% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out least distorting triangle to triangle morphing for one
%traingle.

close all;
clear;

P = [0, 0; 1, 1; 2, 0]; %input triangle
Q = [0,0; 0,1; 1,0]; %goal triangle
F=[1,2,3]; %describes connection between verices


Px = zeros(6);
Qx = zeros(6,1);

for i =1:3
    Px(2*(i-1)+1,:)= [P(i,1), P(i,2), 1, 0,0,0];
    Px(2*(i-1)+2,:) = [0,0,0, P(i,1), P(i,2), 1];
    Qx(2*(i-1)+1) = Q(i,1);
    Qx(2*(i-1)+2) = Q(i,2);
end

Al = Px\Qx; %solve for matrix of roations and scaling.
A = [Al(1), Al(2); Al(4), Al(5)];

[V,D,U] = svd(A); %decompose using single value decomposition.

S = U*D*U'; %symmetric matrix
R = V*U'; %rotation matrix.

interpolations=250;
for i=1: interpolations %display morphing.
    Rt = [(R(1,1)-1)/interpolations*(i-1)+1, (R(1,2))/interpolations*(i-1); (R(2,1))/interpolations*(i-1), (R(2,2)-1)/interpolations*(i-1)+1];
    At = Rt*((1-1/interpolations*(i-1))*eye(2) +  1/interpolations*(i-1)*S);
    V = [(At*P(1,:)')'; (At*P(2,:)')'; (At*P(3,:)')'];
    trimesh(F, V(:,1), V(:,2));
    axis([-1,2.5,-1,1.5]);
    drawnow;
end