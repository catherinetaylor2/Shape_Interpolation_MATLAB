%Rigid as possible animation

close all;
clear;

%Load in keyframes.

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

figure('units','normalized','outerposition',[0 0 1 1]);

q=cell(5,length(FV1),1);
S=cell(5,length(FV1),1);
angles = cell(5, length(FV1),1);
inv_px=cell(1,length(FV1),1); %this will be used to find coefficients for b_ij.
A = zeros(4*length(FV1), 2*length(V1)-2);
b=zeros(4*length(FV1),1);
Px = zeros(6);
Qx = zeros(6,1);
interpolations = 100;
q0 = [1,0,0,0]; %initial quaternions.

fixed_vertices=[V1(1,1), V1(1,2); V2(1,1), V2(1,2); V3(1,1), V3(1,2); V4(1,1), V4(1,2); V5(1,1), V5(1,2); V6(1,1), V6(1,2)];

for i=1:length(FV1) %calculate A for each triangle. 
    P1 = [V1(FV1(i,1),1), V1(FV1(i,1),2); V1(FV1(i,2),1), V1(FV1(i,2),2); V1(FV1(i,3),1), V1(FV1(i,3),2)]; %build each source triangle
    P2 = [V2(FV1(i,1),1), V2(FV1(i,1),2); V2(FV1(i,2),1), V2(FV1(i,2),2); V2(FV1(i,3),1), V2(FV1(i,3),2)];
    P3 = [V3(FV1(i,1),1), V3(FV1(i,1),2); V3(FV1(i,2),1), V3(FV1(i,2),2); V3(FV1(i,3),1), V3(FV1(i,3),2)];
    P4 = [V4(FV1(i,1),1), V4(FV1(i,1),2); V4(FV1(i,2),1), V4(FV1(i,2),2); V4(FV1(i,3),1), V4(FV1(i,3),2)];
    P5 = [V5(FV1(i,1),1), V5(FV1(i,1),2); V5(FV1(i,2),1), V5(FV1(i,2),2); V5(FV1(i,3),1), V5(FV1(i,3),2)];
    P6 = [V6(FV1(i,1),1), V6(FV1(i,1),2); V6(FV1(i,2),1), V6(FV1(i,2),2); V6(FV1(i,3),1), V6(FV1(i,3),2)];

    for j =1:3
        Px(2*(j-1)+1,:)= [P1(j,1), P1(j,2), 1, 0,0,0];
        Px(2*(j-1)+2,:) = [0,0,0, P1(j,1), P1(j,2), 1];
        Qx(2*(j-1)+1) = P2(j,1);
        Qx(2*(j-1)+2) = P2(j,2);
    end
    inv_px{1,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    S{1,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    z = (R(1,2)-R(2,1))/(4*w);
    q{1,i} =[w,0,0,z];
    angles{1,i} = acos(dot(q0,q{1,i}));
    
    for j =1:3
        Px(2*(j-1)+1,:)= [P2(j,1), P2(j,2), 1, 0,0,0];
        Px(2*(j-1)+2,:) = [0,0,0, P2(j,1), P2(j,2), 1];
        Qx(2*(j-1)+1) = P3(j,1);
        Qx(2*(j-1)+2) = P3(j,2);
    end
    inv_px{2,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    S{2,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    z = (R(1,2)-R(2,1))/(4*w);
    q{2,i} =[w,0,0,z];
    angles{2,i} = acos(dot(q0,q{2,i}));
    
    for j =1:3
        Px(2*(j-1)+1,:)= [P3(j,1), P3(j,2), 1, 0,0,0];
        Px(2*(j-1)+2,:) = [0,0,0, P3(j,1), P3(j,2), 1];
        Qx(2*(j-1)+1) = P4(j,1);
        Qx(2*(j-1)+2) = P4(j,2);
    end
    inv_px{3,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    Ut=U';
    S{3,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    z = (R(1,2)-R(2,1))/(4*w);
    q{3,i} =[w,0,0,z];
    angles{3,i} = acos(dot(q0,q{3,i}));
    
    for j =1:3
        Px(2*(j-1)+1,:)= [P4(j,1), P4(j,2), 1, 0,0,0];
        Px(2*(j-1)+2,:) = [0,0,0, P4(j,1), P4(j,2), 1];
        Qx(2*(j-1)+1) = P5(j,1);
        Qx(2*(j-1)+2) = P5(j,2);
    end
    inv_px{4,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    S{4,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    z = (R(1,2)-R(2,1))/(4*w);
    q{4,i} =[w,0,0,z];
    angles{4,i} = acos(dot(q0,q{4,i}));
    
    for j =1:3
        Px(2*(j-1)+1,:)= [P5(j,1), P5(j,2), 1, 0,0,0];
        Px(2*(j-1)+2,:) = [0,0,0, P5(j,1), P5(j,2), 1];
        Qx(2*(j-1)+1) = P6(j,1);
        Qx(2*(j-1)+2) = P6(j,2);
    end
    inv_px{5,i} = inv(Px);
    Al = Px\Qx;
    Ai= [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.
    [V,D,U] = svd(Ai); %decompose using single value decomposition.
    S{5,i} = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    w = sqrt((1+R(1,1)+R(2,2)+1))/2; %quaternion calculations
    z = (R(1,2)-R(2,1))/(4*w);
    q{5,i} =[w,0,0,z];
    angles{5,i} = acos(dot(q0,q{5,i}));
        
end


for in=1:5
    for l=1:interpolations+1
        t=1/interpolations*(l-1);
        
        for i =1:length(FV1)
            
            q_t = 1/sin(angles{in,i})*(sin((1-t)*angles{in,i}))*q0 + sin(t*angles{in,i})/sin(angles{in,i})*q{in,i}; %slerp
            Rot_t= [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2];
            At = Rot_t*((1-t)*eye(2) +  t*S{in,i});
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
        title('As-rigid-as-possible');
        axis([-20 20 -15 15]);
        drawnow;
    end
end