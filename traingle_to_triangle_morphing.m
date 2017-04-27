% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out least distorting triangle to triangle morphing for one
%traingle.

close all;
clear;

P = [10.4569040000000,-11.7576220000000;9.27648600000000,-12.8221320000000;9.27648700000000,-11.7575890000000];%[0, 0; 1, 1; 2, 0]; %input triangle
Q = [8.72115000000000,-13.8898000000000;7.40367000000000,-14.7790000000000;7.55230000000000,-13.7249000000000];[0,0; 0,1; 1,0]; %goal triangle
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
Ut=U';

S = U*D*U'; %symmetric matrix
R = V*U'; %rotation matrix.
figure('units','normalized','outerposition',[0 0 1 1]);
suptitle('Triangle-to-triangle morphing');
interpolations=250;
for i=1: interpolations+1 %display morphing.
    if (i == interpolations/2+1)
       y=9; 
    end
    Rt = [(R(1,1)-1)/interpolations*(i-1)+1, (R(1,2))/interpolations*(i-1); (R(2,1))/interpolations*(i-1), (R(2,2)-1)/interpolations*(i-1)+1];
    Rat = [(V(1,1)-1)/interpolations*(i-1)+1, (V(1,2))/interpolations*(i-1); (V(2,1))/interpolations*(i-1), (V(2,2)-1)/interpolations*(i-1)+1];
    Rbt = [(Ut(1,1)-1)/interpolations*(i-1)+1, (Ut(1,2))/interpolations*(i-1); (Ut(2,1))/interpolations*(i-1), (Ut(2,2)-1)/interpolations*(i-1)+1];
    At = Rt*((1-1/interpolations*(i-1))*eye(2) +  1/interpolations*(i-1)*S);
    At2 = Rat*((1-1/interpolations*(i-1))*eye(2) +  1/interpolations*(i-1)*D)*Rbt;
    At3 = (1-1/interpolations*(i-1))*eye(2) +  1/interpolations*(i-1)*A;
    V1 = [(At*P(1,:)')'; (At*P(2,:)')'; (At*P(3,:)')']; %1 rotation matrix and 1 symmetric matrx.
    V2 = [(At2*P(1,:)')'; (At2*P(2,:)')'; (At2*P(3,:)')']; %SVD
    V3 = [(At3*P(1,:)')'; (At3*P(2,:)')'; (At3*P(3,:)')']; %linear interp
    subplot(1,3,3);
    trimesh(F, V1(:,1), V1(:,2));
    xlabel('Rotation and Symmetric Matrix')
    axis([-0.5,2.5,-1,1.5]);
    subplot(1,3,2);
    trimesh(F, V2(:,1), V2(:,2));
    axis([-0.5,2.5,-1,1.5]);
    xlabel('SVD Decomposition');
    subplot(1,3,1)
    trimesh(F, V3(:,1), V3(:,2));
    xlabel('Linear Interpolation');
    axis([-0.5,2.5,-1,1.5]);
    
    drawnow;
end