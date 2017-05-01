%carries out mesh interp solving using the constraint method.

close all;
clear;

obj1 = readObj('man.obj'); %reads object file and stores vertices and faces.
FV1 = obj1.f.v;
V1 = obj1.v;
% a = V1(:,1); %deals with mesh data.
%  V1(:,2) = -V1(:,2);
% V1(:,2) = -a;  

obj2 = readObj('man2.obj');
FV2 = obj2.f.v;
V2 = obj2.v;
% V2=-V2;
% a = V2(:,1); %deals with mesh data.
% V2(:,1) = -V2(:,3);
% V2(:,2) = -a;  
% V2(:,1) = -V2(:,1);

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
inv_px=cell(1,length(FV1),1); %this will be used to find coefficients for b_ij.
A = zeros(4*length(FV1)+2, 2*length(V1));
b=zeros(4*length(FV1)+2,1);
Px = zeros(6);
Qx = zeros(6,1);
interpolations = 50;


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
end
for l=1:interpolations+1 %vary t between 0 and 1 to get deformation. 
    t=1/interpolations*(l-1);

    for i =1:length(FV1)
        [V,D,U] = svd(Ai{i}); %decompose using single value decomposition.
        Ut=U';
        S = U*D*U'; %symmetric matrix
        Rot = V*U'; %rotation matrix.
        
        Rot_t = [(Rot(1,1)-1)*t+1, (Rot(1,2))*t; (Rot(2,1))*t, (Rot(2,2)-1)*t+1];
        At = Rot_t*((1-t)*eye(2) +  t*S);
  
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
%     axis([-20 20 -15 15]);
 axis([-2 2 -2 2]);
    subplot(1,2,1)
    V_new2 = (1-t)*V1 + t*V2;
    trimesh(FV1, V_new2(:,1), V_new2(:,2));
    title('Linear');
%     axis([-20 20 -15 15]);
     axis([-2 2 -2 2]);
    drawnow;
end