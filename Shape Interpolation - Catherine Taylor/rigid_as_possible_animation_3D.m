%3D rigid as poss animation
close all;
clear;

obj1 = readObj('cube1.obj'); %reads object file and stores vertices and faces.
obj2 = readObj('cube2.obj');
obj3 = readObj('cube3.obj');
obj4 = readObj('cube4.obj');
FV1 = obj1.f.v;
V1 = obj1.v;
V2 = obj2.v;
V3 = obj3.v;
V4 = obj4.v;

figure

%Calculate diagonal of bounding box to find alpha value 
V_min = [min(V1(:,1)), min(V1(:,2)), min(V1(:,3))];
V_max = [max(V1(:,1)), max(V1(:,2)), max(V1(:,3))];
box_length = sqrt(dot(V_min - V_max, V_min - V_max));
alpha = sqrt(1/box_length);

%Set up initial values and cells.

total_interpolations=100;
q0 = [1,0,0,0]; %initial quaternions.

q = cell(3, length(FV1),1);
angle = cell(3, length(FV1),1);
S = cell(3, length(FV1),1);
T=cell(3, length(FV1),1);
inv_kx = cell(3, length(FV1),1);
area = cell(3, length(FV1),1);
b=zeros(12*length(FV1), 1);
bx=zeros(4*length(FV1), 1);
by=zeros(4*length(FV1), 1);
bz=zeros(4*length(FV1), 1);
K = cell(3, 1,1); 
K{1} = zeros(4*length(FV1), length(FV1)+length(V1));
K{2} = zeros(4*length(FV1), length(FV1)+length(V1));
K{3} = zeros(4*length(FV1), length(FV1)+length(V1));

for i=1:length(FV1)
    v1 = [V1(FV1(i,1),1), V1(FV1(i,1),2),V1(FV1(i,1),3)]' ;
    v2 = [V1(FV1(i,2),1), V1(FV1(i,2),2), V1(FV1(i,2),3)]' ;
    v3 = [V1(FV1(i,3),1),V1(FV1(i,3),2),V1(FV1(i,3),3)]' ;
    v4 = 1/3*(v1+v2+v3) + cross(v2-v1, v3-v2)/sqrt(norm(cross(v2-v1, v3-v2)));
    V = [v1-v4, v2-v4, v3-v4];
    
    kx = [v1' 1; v2', 1; v3', 1; v4', 1];
    inv_kx{1,i} = inv(kx);
    
    u1 = [V2(FV1(i,1),1), V2(FV1(i,1),2),V2(FV1(i,1),3)]' ;
    u2 = [V2(FV1(i,2),1), V2(FV1(i,2),2), V2(FV1(i,2),3)]' ;
    u3 = [V2(FV1(i,3),1),V2(FV1(i,3),2),V2(FV1(i,3),3)]' ;
    u4 = 1/3*(u1+u2+u3) + cross(u2-u1, u3-u2)/sqrt(norm(cross(u2-u1, u3-u2)));
    U = [u1-u4, u2-u4, u3-u4];
    
    M = U/V;
    T{1,i} = u1-M*v1;
    
    [V,D,U] = svd(M);
    R = V*U';
    S{1,i} = U*D*U';
    
    area{1,i} = norm(cross(v2-v1,v3-v1))/2;

    for k=1:3
        K{1}(4*(i-1)+1:4*(i-1)+4, FV1(i,k)) = area{1,i}*inv_kx{1,i}(1:4,k);
    end

    K{1}(4*(i-1)+1:4*(i-1)+4, length(V1)+i) = area{1,i}*inv_kx{1,i}(1:4, 4);
    
    q{1,i} =Matrix_to_quaternion( R );
    angle{1, i} = acos(dot(q0,q{1,i}));
    
    v1=u1;
    v2=u2;
    v3=u3;
    v4=u4;
    V = [v1-v4, v2-v4, v3-v4];
    kx = [v1' 1; v2', 1; v3', 1; v4', 1];
    inv_kx{2,i} = inv(kx);
    
    u1 = [V3(FV1(i,1),1), V3(FV1(i,1),2), V3(FV1(i,1),3)]' ;
    u2 = [V3(FV1(i,2),1), V3(FV1(i,2),2), V3(FV1(i,2),3)]' ;
    u3 = [V3(FV1(i,3),1), V3(FV1(i,3),2), V3(FV1(i,3),3)]' ;
    u4 = 1/3*(u1+u2+u3) + cross(u2-u1, u3-u2)/sqrt(norm(cross(u2-u1, u3-u2)));
    U = [u1-u4, u2-u4, u3-u4];
    
    M = U/V;
    T{2,i} = u1-M*v1;
    
    [V,D,U] = svd(M);
    R = V*U';
    S{2,i} = U*D*U';
    
    area{2,i} = norm(cross(v2-v1,v3-v1))/2;
    
    for k=1:3
        K{2}(4*(i-1)+1:4*(i-1)+4, FV1(i,k)) = area{2,i}*inv_kx{2,i}(1:4,k);
    end

    K{2}(4*(i-1)+1:4*(i-1)+4, length(V1)+i) = area{2,i}*inv_kx{2,i}(1:4, 4);
    
    q{2,i} =Matrix_to_quaternion( R );
    angle{2, i} = acos(dot(q0,q{2,i}));
    
    v1=u1;
    v2=u2;
    v3=u3;
    v4=u4;
    V = [v1-v4, v2-v4, v3-v4];
    kx = [v1' 1; v2', 1; v3', 1; v4', 1];
    inv_kx{3,i} = inv(kx);
    
    u1 = [V4(FV1(i,1),1), V4(FV1(i,1),2), V4(FV1(i,1),3)]' ;
    u2 = [V4(FV1(i,2),1), V4(FV1(i,2),2), V4(FV1(i,2),3)]' ;
    u3 = [V4(FV1(i,3),1), V4(FV1(i,3),2), V4(FV1(i,3),3)]' ;
    u4 = 1/3*(u1+u2+u3) + cross(u2-u1, u3-u2)/sqrt(norm(cross(u2-u1, u3-u2)));
    U = [u1-u4, u2-u4, u3-u4];
    
    M = U/V;
    T{3,i} = u1-M*v1;
    
    [V,D,U] = svd(M);
    R = V*U';
    S{3,i} = U*D*U';
    
    area{3,i} = norm(cross(v2-v1,v3-v1))/2;
    
    for k=1:3
        K{3}(4*(i-1)+1:4*(i-1)+4, FV1(i,k)) = area{3,i}*inv_kx{3,i}(1:4,k);
    end

    K{3}(4*(i-1)+1:4*(i-1)+4, length(V1)+i) = area{3,i}*inv_kx{3,i}(1:4, 4);

    q{3,i} =Matrix_to_quaternion( R );
    angle{3, i} = acos(dot(q0,q{3,i}));
end

for in = 1:3
    for j=1:total_interpolations
        t=1/total_interpolations*(j-1);
        
        for i=1:length(FV1)
            
            S_t = (1-t)*eye(3) + t*S{in,i};
            T_t = t*T{in,i};
            
            q_t = 1/sin(angle{in,i})*(sin((1-t)*angle{in,i}))*q0 + sin(t*angle{in,i})/sin(angle{in,i})*q{in,i}; %slerp
            Rot_t = quaternion_to_matrix(q_t);
            M_t = Rot_t*S_t;
            
            bx(4*(i-1)+1:4*(i-1)+4) = area{in,i}*[M_t(1,1), M_t(1,2), M_t(1,3), alpha*T_t(1)];
            by(4*(i-1)+1:4*(i-1)+4) = area{in,i}*[M_t(2,1), M_t(2,2), M_t(2,3), alpha*T_t(2)];
            bz(4*(i-1)+1:4*(i-1)+4) = area{in,i}*[M_t(3,1), M_t(3,2), M_t(3,3), alpha*T_t(3)];
        end
        
        V_new_x = (K{in}'*K{in})\K{in}'*bx;
        V_new_y = (K{in}'*K{in})\K{in}'*by;
        V_new_z = (K{in}'*K{in})\K{in}'*bz;
        
        V_intermediate = [V_new_x(1:length(V1)), V_new_y(1:length(V1)), V_new_z(1:length(V1))];
        
        trimesh(FV1(:, 1:3), V_intermediate(:,1), V_intermediate(:,2), V_intermediate(:,3));
        xlabel('x'), ylabel('y'), zlabel('z')
        title('As-rigid-as-possible 3D Interpolations')
        axis([-2, 2, 0, 4,-2,2]);
        drawnow;
        
    end
end