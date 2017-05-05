% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out as-rigid-as-possible shape interpolation using 2D meshes.
%Requires two meshes with 1-1 correspondance between triangles. Solves by
%reducing A and V.

close all;
clear;

obj1 = readObj('man.obj'); %reads object file and stores vertices and faces.
obj2 = readObj('man2.obj');

FV1 = obj1.f.v;
V1 = obj1.v;
V2 = obj2.v;

figure
subplot(1,2,1)
trimesh(FV1, V1(:,1), V1(:,2));
title('Input Mesh')
hold on
subplot(1,2,2);
trimesh(FV1, V2(:,1), V2(:,2));
title('Goal Mesh')

figure('units','normalized','outerposition',[0 0 1 1]);
suptitle('Shape Interpolation');

q=cell(1,length(FV1),1);
angle = cell(1, length(FV1),1);
Symmetric_Matrices=cell(1,length(FV1),1);
inv_px=cell(1,length(FV1),1); %this will be used to find coefficients for b_ij.
A = zeros(4*length(FV1), 2*length(V1)-2);
b=zeros(4*length(FV1),1);
Px = zeros(6);
Qx = zeros(6,1);
interpolations = 100;
q0 = [1,0,0,0]; %initial quaternions.

for i=1:length(FV1) %calculate A for each triangle. 
   for j =1:3
        Px(2*(j-1)+1:2*(j-1)+2,:)= [V1(FV1(i,j),1), V1(FV1(i,j),2), 1, 0,0,0; 0,0,0, V1(FV1(i,j),1), V1(FV1(i,j),2), 1];
        Qx(2*(j-1)+1:2*(j-1)+2) = V2(FV1(i,j),1:2);
   end
    
    inv_px{i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    Symmetric_Matrices{i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    q{i} =[w,0,0,(R(1,2)-R(2,1))/(4*w)];
    angle{i} = acos(dot(q0,q{i}));
        
end

for l=1:interpolations+1 %vary t between 0 and 1 to get deformation. 
    t=1/interpolations*(l-1);
    v1=(1-t)*V1(1,1)+t*V2(1,1);  %constraints for fixed vertex. 
    v2= (1-t)*V1(1,2)+t*V2(1,2);
    
    for i =1:length(FV1) 
   
        q_t = 1/sin(angle{i})*(sin((1-t)*angle{i}))*q0 + sin(t*angle{i})/sin(angle{i})*q{i}; %slerp
        Rot_t= [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2];
        At = Rot_t*((1-t)*eye(2) +  t*Symmetric_Matrices{i});
        
        if((FV1(i,1)==1))
            b(4*(i-1)+1:4*(i-1)+4)=[At(1,1)- (inv_px{i}(1,1)*v1+ inv_px{i}(1,2)*v2), At(1,2)- (inv_px{i}(2,1)*v1+ inv_px{i}(2,2)*v2), At(2,1)- (inv_px{i}(4,1)*v1 + inv_px{i}(4,2)*v2), At(2,2)- (inv_px{i}(5,1)*v1+ inv_px{i}(5,2)*v2)]'; %build up matrix b, for min ||Ax-b||.
        elseif ((FV1(i,2)==1))
            b(4*(i-1)+1:4*(i-1)+4)=[At(1,1)- (inv_px{i}(1,3)*v1+ inv_px{i}(1,4)*v2), At(1,2)- (inv_px{i}(2,3)*v1+ inv_px{i}(2,4)*v2), At(2,1)- (inv_px{i}(4,3)*v1 + inv_px{i}(4,4)*v2), At(2,2)- (inv_px{i}(5,3)*v1+ inv_px{i}(5,4)*v2)]'; %build up matrix b, for min ||Ax-b||.
        elseif ((FV1(i,3)==1))
            b(4*(i-1)+1:4*(i-1)+4)=[At(1,1)- (inv_px{i}(1,5)*v1+ inv_px{i}(1,6)*v2), At(1,2)- (inv_px{i}(2,5)*v1+ inv_px{i}(2,6)*v2), At(2,1)- (inv_px{i}(4,5)*v1 + inv_px{i}(4,6)*v2), At(2,2)-(inv_px{i}(5,5)*v1+ inv_px{i}(5,6)*v2)]'; %build up matrix b, for min ||Ax-b||.
        else
            b(4*(i-1)+1:4*(i-1)+4)=[At(1,1), At(1,2), At(2,1), At(2,2)]'; %build up matrix b, for min ||Ax-b||.
        end
        
        for k=1:3 %build up matrix A, for min ||Ax-b||.
            if(FV1(i,k)~=1)
            A(4*(i-1)+1, 2*(FV1(i,k)-2)+1:2*(FV1(i,k)-2)+2) = inv_px{i}(1,2*(k-1)+1:2*(k-1)+2);
            A(4*(i-1)+2, 2*(FV1(i,k)-2)+1:2*(FV1(i,k)-2)+2) = inv_px{i}(2,2*(k-1)+1:2*(k-1)+2);
            A(4*(i-1)+3, 2*(FV1(i,k)-2)+1:2*(FV1(i,k)-2)+2) = inv_px{i}(4,2*(k-1)+1:2*(k-1)+2);
            A(4*(i-1)+4, 2*(FV1(i,k)-2)+1:2*(FV1(i,k)-2)+2) = inv_px{i}(5,2*(k-1)+1:2*(k-1)+2);
            end
        end
    end
    
    V_new = (A'*A)'\A'*b; %solve for optimal new vertex positions.
    V_1=zeros(length(V1),2);
    for i =1:length(V_new)
        if (mod(i,2)==0)
            V_1(1+i/2, 2) = V_new(i);
        else
            V_1(1+(i-1)/2+1,1) = V_new(i);
        end
    end
    V_1(1,1)=v1;  %constraints for fixed vertex.
    V_1(1,2)= v2;
    
    subplot(1,2,2) %display results.
    trimesh(FV1, V_1(:,1), V_1(:,2));
    title('As-rigid-as-possible');
    %     axis([-20 20 -15 15]);
    axis([-2 2 -2 2]);
    subplot(1,2,1)
    V_new2 = (1-t)*V1 + t*V2;
    trimesh(FV1, V_new2(:,1), V_new2(:,2));
    title('Linear');
    %   axis([-20 20 -15 15]);
    axis([-2 2 -2 2]);
    drawnow;
end