% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out as-rigid-as-possible shape interpolation using 2D meshes.
%Requires two meshes with 1-1 correspondance between triangles. Solves by
%adding a constraint to the cost function.

close all;
clear;

obj1 = readObj('keyframe1.obj'); %reads object file and stores vertices and faces.
FV1 = obj1.f.v;
V1 = obj1.v;

obj2 = readObj('keyframe2.obj');
FV2 = obj2.f.v;
V2 = obj2.v;


figure
subplot(1,2,1)
trimesh(FV1, V1(:,1), V1(:,2));
title('Input Mesh')
hold on
subplot(1,2,2);
trimesh(FV2, V2(:,1), V2(:,2));
title('Goal Mesh')

figure('units','normalized','outerposition',[0 0 1 1]);
suptitle('Shape Interpolation');

Ai=cell(1,length(FV1),1);
Rot=cell(1,length(FV1),1);
S=cell(1,length(FV1),1);
inv_px=cell(1,length(FV1),1); %this will be used to find coefficients for b_ij.
A = zeros(4*length(FV1)+2, 2*length(V1));
b=zeros(4*length(FV1)+2,1);
Px = zeros(6);
Qx = zeros(6,1);
interpolations = 50;
q0 = [1,0,0,0]; %initial quaternions.


for i=1:length(FV1) %calculate A for each triangle. 
    P = [V1(FV1(i,1),1), V1(FV1(i,1),2); V1(FV1(i,2),1), V1(FV1(i,2),2); V1(FV1(i,3),1), V1(FV1(i,3),2)]; %build each source triangle
    Q = [V2(FV2(i,1),1), V2(FV2(i,1),2); V2(FV2(i,2),1), V2(FV2(i,2),2); V2(FV2(i,3),1), V2(FV2(i,3),2)]; %build each target triangle
    
    for j =1:3
        Px(2*(j-1)+1,:)= [P(j,1), P(j,2), 1, 0,0,0];
        Px(2*(j-1)+2,:) = [0,0,0, P(j,1), P(j,2), 1];
        Qx(2*(j-1)+1) = Q(j,1);
        Qx(2*(j-1)+2) = Q(j,2);
    end
    inv_px{i} = inv(Px);
    Al = Px\Qx;
    Ai{i} = [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai{i}); %decompose using single value decomposition.
    Ut=U';
    S{i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    Rot{i} = [R(1,1), R(1,2), 0; R(2,1), R(2,2), 0; 0,0,1]; %put in 3D to use quaternions.
end
for l=1:interpolations+1 %vary t between 0 and 1 to get deformation. 
    t=1/interpolations*(l-1);

    for i =1:length(FV1)

        
        w = sqrt((1+Rot{i}(1,1)+Rot{i}(2,2)+Rot{i}(3,3)))/2; %quaternion calculations
        x = (Rot{i}(2,3) - Rot{i}(3,2))/(4*w);
        y = (Rot{i}(3,1) - Rot{i}(1,3))/(4*w);
        z = (Rot{i}(1,2)-Rot{i}(2,1))/(4*w);
        q =[w,x,y,z];
        angle = acos(dot(q0,q));
    
        q_t = 1/sin(angle)*(sin((1-t)*angle))*q0 + sin(t*angle)/sin(angle)*q; %slerp
        Rot_t_full = [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4), 2*q_t(4)*q_t(2)- 2*q_t(1)*q_t(3); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2, 2*q_t(3)*q_t(4) + 2*q_t(1)*q_t(2); 2*q_t(2)*q_t(4)+2*q_t(1)*q_t(3), 2*q_t(3)*q_t(4)-2*q_t(1)*q_t(2), 1- 2*q_t(2)^2 - 2*q_t(3)^2];
        Rot_t = Rot_t_full(1:2, 1:2);
        At = Rot_t*((1-t)*eye(2) +  t*S{i});
  
        b(4*(i-1)+1:4*(i-1)+4)=[At(1,1), At(1,2), At(2,1), At(2,2)]'; %build up matrix b, for min ||Ax-b||.

        for k=1:3 %build up matrix A, for min ||Ax-b||.

            A(4*(i-1)+1, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{i}(1,2*(k-1)+1:2*(k-1)+2);
            A(4*(i-1)+2, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{i}(2,2*(k-1)+1:2*(k-1)+2);
            A(4*(i-1)+3, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{i}(4,2*(k-1)+1:2*(k-1)+2);
            A(4*(i-1)+4, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{i}(5,2*(k-1)+1:2*(k-1)+2);

        end
    end
    b(4*length(FV1)+1)= (1-t)*V1(1,1)+t*V2(1,1);  %constraints for fixed vertex. 
    b(4*length(FV1)+2)= (1-t)*V1(1,2)+t*V2(1,2);
    A(4*length(FV1)+1,1) = 1;
    A(4*length(FV1)+2,2)=1;


    V_new = (A'*A)'\A'*b; %solve for optimal new vertex positions.
    V_1=zeros(length(V1),2);
    for i =1:length(V_new)
        if (mod(i,2)==0)
            V_1(i/2, 2) = V_new(i);
        else
            V_1((i-1)/2+1,1) = V_new(i);
        end
    end

    subplot(1,2,2) %display results.
    trimesh(FV1, V_1(:,1), V_1(:,2));
    title('As-rigid-as-possible');
    axis([-20 20 -20 15]);
%  axis([-2 2 -2 2]);
    subplot(1,2,1)
    V_new2 = (1-t)*V1 + t*V2;
    trimesh(FV1, V_new2(:,1), V_new2(:,2));
    title('Linear');
 axis([-20 20 -20 15]);
%      axis([-2 2 -2 2]);
    drawnow;
end