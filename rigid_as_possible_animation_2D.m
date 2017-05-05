% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

% Carries out as-rigid-as-possible shape interpolation using 3D meshes and
% produces a short animation.

close all;
clear;

obj1 = readObj('dino.obj'); %Load in keyframes.
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
fixed_vertices=[V1(1,1), V1(1,2); V2(1,1), V2(1,2); V3(1,1), V3(1,2); V4(1,1), V4(1,2); V5(1,1), V5(1,2); V6(1,1), V6(1,2)];

figure('units','normalized','outerposition',[0 0 1 1]);
total_interpolations = 50;
q0 = [1,0,0,0]; %initial quaternions.

quaternions=cell(5,length(FV1),1);
Symmetric_matrices=cell(5,length(FV1),1);
angles = cell(5, length(FV1),1);
inv_px=cell(1,length(FV1),1); %this will be used to find coefficients for b_ij.
A = zeros(4*length(FV1), 2*length(V1)-2);
b=zeros(4*length(FV1),1);
Px = zeros(6);
Qx = zeros(6,1);

for i=1:length(FV1) %calculate A for each triangle. 
    for j =1:3
        Px(2*(j-1)+1:2*(j-1)+2,:)= [V1(FV1(i,j),1), V1(FV1(i,j),2), 1, 0,0,0; 0,0,0, V1(FV1(i,j),1), V1(FV1(i,j),2), 1];
        Qx(2*(j-1)+1:2*(j-1)+2) = V2(FV1(i,j),1:2);
    end
    
    inv_px{1,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    Symmetric_matrices{1,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    quaternions{1,i} =[w,0,0,(R(1,2)-R(2,1))/(4*w)];
    angles{1,i} = acos(dot(q0,quaternions{1,i}));
    
    for j =1:3
        Px(2*(j-1)+1:2*(j-1)+2,:)= [V2(FV1(i,j),1), V2(FV1(i,j),2), 1, 0,0,0; 0,0,0, V2(FV1(i,j),1), V2(FV1(i,j),2), 1];
        Qx(2*(j-1)+1:2*(j-1)+2) = V3(FV1(i,j),1:2);
    end
    
    inv_px{2,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    Symmetric_matrices{2,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    quaternions{2,i} =[w,0,0,(R(1,2)-R(2,1))/(4*w)];
    angles{2,i} = acos(dot(q0,quaternions{2,i}));

    q_t = sin(angles{2,i})/sin(angles{2,i})*quaternions{2,i}; %slerp
    Rot_t= [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2];
    At = Rot_t*Symmetric_matrices{2,i};
    b(4*(i-1)+1:4*(i-1)+4)=[At(1,1), At(1,2), At(2,1), At(2,2)]'; %build up matrix b, for min ||Ax-b||.
            
    for k=1:3 %build up matrix A, for min ||Ax-b||.
        A(4*(i-1)+1, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{2,i}(1,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+2, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{2,i}(2,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+3, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{2,i}(4,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+4, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{2, i}(5,2*(k-1)+1:2*(k-1)+2);
    end
end
        
b(4*length(FV1)+1:4*length(FV1)+2)= fixed_vertices(3,1:2);  %constraints for fixed vertex.
A(4*length(FV1)+1,1) = 1;
A(4*length(FV1)+2,2)=1;

%Find end points so they become starting point for next interpolation.
V_new = (A'*A)'\A'*b; %solve for optimal new vertex positions.
V_3=zeros(length(V1),2);
for i =1:length(V_new)
    if (mod(i,2)==0)
        V_3(i/2, 2) = V_new(i);
    else
        V_3((i-1)/2+1,1) = V_new(i);
    end
end

for i=1:length(FV1) %calculate A for each triangle. 
    for j =1:3
        Px(2*(j-1)+1:2*(j-1)+2,:)= [V_3(FV1(i,j),1), V_3(FV1(i,j),2), 1, 0,0,0;0,0,0, V_3(FV1(i,j),1), V_3(FV1(i,j),2), 1];
        Qx(2*(j-1)+1:2*(j-1)+2) = V4(FV1(i,j),1:2);
    end
    
    inv_px{3,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    Symmetric_matrices{3,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    quaternions{3,i} =[w,0,0,(R(1,2)-R(2,1))/(4*w)];
    angles{3,i} = acos(dot(q0,quaternions{3,i}));

    q_t = sin(angles{3,i})/sin(angles{3,i})*quaternions{3,i}; %slerp
    Rot_t= [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2];
    At = Rot_t*Symmetric_matrices{3,i};
    b(4*(i-1)+1:4*(i-1)+4)=[At(1,1), At(1,2), At(2,1), At(2,2)]'; %build up matrix b, for min ||Ax-b||.
            
    for k=1:3 %build up matrix A, for min ||Ax-b||.
        A(4*(i-1)+1, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{3,i}(1,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+2, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{3,i}(2,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+3, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{3,i}(4,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+4, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{3, i}(5,2*(k-1)+1:2*(k-1)+2); 
    end
end
        
b(4*length(FV1)+1:4*length(FV1)+2)= fixed_vertices(3+1,1:2);  %constraints for fixed vertex.
A(4*length(FV1)+1,1) = 1;
A(4*length(FV1)+2,2)=1;
        
V_new = (A'*A)'\A'*b; %solve for optimal new vertex positions.
V_4=zeros(length(V1),2);
for i =1:length(V_new)
    if (mod(i,2)==0)
        V_4(i/2, 2) = V_new(i);
    else
        V_4((i-1)/2+1,1) = V_new(i);
    end
end

for i=1:length(FV1) %calculate A for each triangle.
    for j =1:3
        Px(2*(j-1)+1:2*(j-1)+2,:)= [V_4(FV1(i,j),1), V_4(FV1(i,j),2), 1, 0,0,0;0,0,0, V_4(FV1(i,j),1), V_4(FV1(i,j),2), 1];
        Qx(2*(j-1)+1:2*(j-1)+2) = V5(FV1(i,j),1:2);
    end
    inv_px{4,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    Symmetric_matrices{4,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    quaternions{4,i} =[w,0,0,(R(1,2)-R(2,1))/(4*w)];
    angles{4,i} = acos(dot(q0,quaternions{4,i}));
    
    q_t = sin(angles{4,i})/sin(angles{4,i})*quaternions{4,i}; %slerp
    Rot_t= [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2];
    At = Rot_t*Symmetric_matrices{4,i};
    b(4*(i-1)+1:4*(i-1)+4)=[At(1,1), At(1,2), At(2,1), At(2,2)]'; %build up matrix b, for min ||Ax-b||.
    
    for k=1:3 %build up matrix A, for min ||Ax-b||.
        A(4*(i-1)+1, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{4,i}(1,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+2, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{4,i}(2,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+3, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{4,i}(4,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+4, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{4, i}(5,2*(k-1)+1:2*(k-1)+2);
        
    end
end

b(4*length(FV1)+1:4*length(FV1)+2)= fixed_vertices(4+1,1:2);  %constraints for fixed vertex.
A(4*length(FV1)+1,1) = 1;
A(4*length(FV1)+2,2)=1;

V_new = (A'*A)'\A'*b; %solve for optimal new vertex positions.
V_5=zeros(length(V1),2);
for i =1:length(V_new)
    if (mod(i,2)==0)
        V_5(i/2, 2) = V_new(i);
    else
        V_5((i-1)/2+1,1) = V_new(i);
    end
end

for i=1:length(FV1) %calculate A for each triangle.
    for j =1:3
        Px(2*(j-1)+1:2*(j-1)+2,:)= [V_5(FV1(i,j),1), V_5(FV1(i,j),2), 1, 0,0,0;0,0,0, V_5(FV1(i,j),1), V_5(FV1(i,j),2), 1];
        Qx(2*(j-1)+1:2*(j-1)+2) = V6(FV1(i,j),1:2);
    end
    
    inv_px{5,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    Symmetric_matrices{5,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    quaternions{5,i} =[w,0,0,(R(1,2)-R(2,1))/(4*w)];
    angles{5,i} = acos(dot(q0,quaternions{5,i}));
end

for in=1:5
    for l=1:total_interpolations
        t=1/total_interpolations*(l-1);
        
        for i =1:length(FV1)
            
            q_t = 1/sin(angles{in,i})*(sin((1-t)*angles{in,i}))*q0 + sin(t*angles{in,i})/sin(angles{in,i})*quaternions{in,i}; %slerp
            Rot_t= [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2];
            At = Rot_t*((1-t)*eye(2) +  t*Symmetric_matrices{in,i});
            b(4*(i-1)+1:4*(i-1)+4)=[At(1,1), At(1,2), At(2,1), At(2,2)]'; %build up matrix b, for min ||Ax-b||.
            
            for k=1:3 %build up matrix A, for min ||Ax-b||.
                A(4*(i-1)+1, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{in,i}(1,2*(k-1)+1:2*(k-1)+2);
                A(4*(i-1)+2, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{in,i}(2,2*(k-1)+1:2*(k-1)+2);
                A(4*(i-1)+3, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{in,i}(4,2*(k-1)+1:2*(k-1)+2);
                A(4*(i-1)+4, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{in, i}(5,2*(k-1)+1:2*(k-1)+2);
                
            end
        end
        
        b(4*length(FV1)+1)= (1-t)*fixed_vertices(in,1)+t*fixed_vertices(in+1,1);  %constraints for fixed vertex.
        b(4*length(FV1)+2)= (1-t)*fixed_vertices(in,2)+t*fixed_vertices(in+1,2);
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
        
        trimesh(FV1, V_1(:,1), V_1(:,2));
        title('As-rigid-as-possible Animation');
        axis([-25 20 -20 15]);
        drawnow;
    end
end