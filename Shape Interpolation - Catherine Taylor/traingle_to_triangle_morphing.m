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
    Px(2*(i-1)+1:2*(i-1)+2,:)= [P(i,1), P(i,2), 1, 0,0,0; 0,0,0, P(i,1), P(i,2), 1];
    Qx(2*(i-1)+1:2*(i-1)+2) = Q(i,1:2);
end

Al = Px\Qx; %solve for matrix of roations and scaling.
A = [Al(1), Al(2); Al(4), Al(5)];

[V,D,U] = svd(A); %decompose using single value decomposition.
Ut=U';

S = U*D*U'; %symmetric matrix
R = V*U'; %rotation matrix.
figure('units','normalized','outerposition',[0 0 1 1]);
suptitle('Triangle-to-triangle morphing');
interpolations=100;

%use quaternions and slerp on rotation matrix;
q0 = [1,0,0,0];
w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
z = (R(1,2)-R(2,1))/(4*w);
q =[w,0,0,z];
angle = acos(dot(q0,q));

w_u = sqrt((1+Ut(1,1)+Ut(2,2)+1))/2;
z_u = (Ut(1,2)-Ut(2,1))/(4*w_u);
q_u = [w_u, 0,0, z_u];
angle_u = acos(dot(q0,q_u));

w_v = sqrt((1+V(1,1)+V(2,2)+1))/2;
z_v = (V(1,2)-V(2,1))/(4*w_v);
q_v = [w_v, 0,0, z_v];
angle_v = acos(dot(q0,q_v));
    
for i=1: interpolations+1 %display morphing.
    t= 1/interpolations*(i-1);
    q_t = 1/sin(angle)*(sin((1-t)*angle))*q0 + sin(t*angle)/sin(angle)*q; %slerp
    q_tu = 1/sin(angle_u)*(sin((1-t)*angle_u))*q0 + sin(t*angle_u)/sin(angle_u)*q_u; %slerp
    q_tv = 1/sin(angle_v)*(sin((1-t)*angle_v))*q0 + sin(t*angle_v)/sin(angle_v)*q_v; %slerp
    
    Rt = [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2]; 
    Rat = [1-2*q_tv(3)^2 - 2*q_tv(4)^2, 2*q_tv(2)*q_tv(3)+2*q_tv(1)*q_tv(4); 2*q_tv(2)*q_tv(3)-2*q_tv(1)*q_tv(4), 1-2*q_tv(2)^2 - 2*q_tv(4)^2]; 
    Rbt = [1-2*q_tu(3)^2 - 2*q_tu(4)^2, 2*q_tu(2)*q_tu(3)+2*q_tu(1)*q_tu(4); 2*q_tu(2)*q_tu(3)-2*q_tu(1)*q_tu(4), 1-2*q_tu(2)^2 - 2*q_tu(4)^2]; 
  
    At = Rt*((1-t)*eye(2) +  t*S);
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