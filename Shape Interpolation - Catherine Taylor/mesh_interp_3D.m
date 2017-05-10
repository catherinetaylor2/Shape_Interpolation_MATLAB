% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out as-rigid-as-possible shape interpolation using 3D meshes.
%Requires two meshes with 1-1 correspondance between triangles. Solves by
%adding a constraint to the cost function.

close all;
clear;

obj1 = readObj('sk1.obj'); %reads object file and stores vertices and faces.
obj2 = readObj('sk2.obj');
FV1 = obj1.f.v;
V1 = obj1.v;
V2 = obj2.v;

figure

%Calculate diagonal of bounding box to find alpha value -------------------
V_min = [min(V2(:,1)), min(V2(:,2)), min(V2(:,3))];
V_max = [max(V2(:,1)), max(V2(:,2)), max(V2(:,3))];
box_length = sqrt(dot(V_min - V_max, V_min - V_max));
alpha = sqrt(1/box_length);
%--------------------------------------------------------------------------
%Set up initial values and cells.

total_interpolations=100;
q0 = [1,0,0,0]; %initial quaternions.

q = cell(1, length(FV1),1);
angle = cell(1, length(FV1),1);
S = cell(1, length(FV1),1);
T=cell(1, length(FV1),1);
inv_kx = cell(1, length(FV1),1);
area = cell(1, length(FV1),1);
b=zeros(12*length(FV1), 1);
bx=zeros(4*length(FV1), 1);
by=zeros(4*length(FV1), 1);
bz=zeros(4*length(FV1), 1);
K = zeros(4*length(FV1), length(FV1)+length(V1));

for i=1:length(FV1)
    v1 = [V1(FV1(i,1),1), V1(FV1(i,1),2),V1(FV1(i,1),3)]' ;
    v2 = [V1(FV1(i,2),1), V1(FV1(i,2),2), V1(FV1(i,2),3)]' ;
    v3 = [V1(FV1(i,3),1),V1(FV1(i,3),2),V1(FV1(i,3),3)]' ;
    v4 = 1/3*(v1+v2+v3) + cross(v2-v1, v3-v2)/sqrt(norm(cross(v2-v1, v3-v2)));
    V = [v1-v4, v2-v4, v3-v4];
    
    kx = [v1' 1; v2', 1; v3', 1; v4', 1];
    inv_kx{i} = inv(kx);
    
    u1 = [V2(FV1(i,1),1), V2(FV1(i,1),2),V2(FV1(i,1),3)]' ;
    u2 = [V2(FV1(i,2),1), V2(FV1(i,2),2), V2(FV1(i,2),3)]' ;
    u3 = [V2(FV1(i,3),1),V2(FV1(i,3),2),V2(FV1(i,3),3)]' ;
    u4 = 1/3*(u1+u2+u3) + cross(u2-u1, u3-u2)/sqrt(norm(cross(u2-u1, u3-u2)));
    U = [u1-u4, u2-u4, u3-u4];
    
    M = U/V;
    T{i} = u1-M*v1;
    
    [V,D,U] = svd(M);
    R = V*U';
    S{i} = U*D*U';
    
    area{i} = norm(cross(v2-v1,v3-v1))/2;
    
    for k=1:3
        K(4*(i-1)+1:4*(i-1)+4, FV1(i,k)) = area{i}*inv_kx{i}(1:4,k);
    end

    K(4*(i-1)+1:4*(i-1)+4, length(V1)+i) = area{i}*inv_kx{i}(1:4, 4);

    q{i} = Matrix_to_quaternion(R);
    angle{i} = acos(dot(q0,q{i}));
        
end

for j= 1:total_interpolations+1
    t=1/total_interpolations*(j-1);
    
    for i=1:length(FV1)
        
        S_t = (1-t)*eye(3) + t*S{i};
        T_t = t*T{i};
        
        q_t = 1/sin(angle{i})*(sin((1-t)*angle{i}))*q0 + sin(t*angle{i})/sin(angle{i})*q{i}; %slerp
        Rot_t = quaternion_to_matrix(q_t);
        M_t = Rot_t*S_t;

        bx(4*(i-1)+1:4*(i-1)+4) = area{i}*[M_t(1,1), M_t(1,2), M_t(1,3), alpha*T_t(1)];
        by(4*(i-1)+1:4*(i-1)+4) = area{i}*[M_t(2,1), M_t(2,2), M_t(2,3), alpha*T_t(2)];
        bz(4*(i-1)+1:4*(i-1)+4) = area{i}*[M_t(3,1), M_t(3,2), M_t(3,3), alpha*T_t(3)];
    end
    
    V_new_x = (K'*K)\K'*bx;
    V_new_y = (K'*K)\K'*by;
    V_new_z = (K'*K)\K'*bz;
    
    V_intermediate = [V_new_x(1:length(V1)), V_new_y(1:length(V1)), V_new_z(1:length(V1))];

    trimesh(FV1(:, 1:3), V_intermediate(:,1), V_intermediate(:,2), V_intermediate(:,3));
    xlabel('x'), ylabel('y'), zlabel('z')
    title('As-rigid-as-possible 3D Interpolations')
    %     axis([-2, 2, -5, ,-2,2]);
    axis([-4, 6, -5, 5, -4,2])
   
    drawnow;
    
end