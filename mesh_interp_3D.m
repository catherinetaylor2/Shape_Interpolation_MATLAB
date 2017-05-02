% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out as-rigid-as-possible shape interpolation using 3D meshes.
%Requires two meshes with 1-1 correspondance between triangles. Solves by
%adding a constraint to the cost function.

close all;
clear;

obj1 = readObj('cube1.obj'); %reads object file and stores vertices and faces.
FV1 = obj1.f.v;
V1 = obj1.v;

obj2 = readObj('cube2.obj'); %reads object file and stores vertices and faces.
FV2 = obj2.f.v;
V2 = obj2.v;

figure
trimesh(FV1(:, 1:3), V1(:,1), V1(:,2), V1(:,3)); 
xlabel('x'), ylabel('y'), zlabel('z');
axis([-1, 1, 1, 3,-1,1]);

%Calculate diagonal of bounding box to find alpha value--------------------
V_min = [min(V1(:,1)), min(V1(:,2)), min(V1(:,3))];
V_max = [max(V1(:,1)), max(V1(:,2)), max(V1(:,3))];
box_length = sqrt(dot(V_min - V_max, V_min - V_max));
alpha = sqrt(1/box_length);
%--------------------------------------------------------------------------
%Set up initial values and cells.

total_interpolations=100;
q0 = [1,0,0,0]; %initial quaternions.

M_t=cell(1, length(FV1), 1);
R = cell(1, length(FV1),1);
S = cell(1, length(FV1),1);
T=cell(1, length(FV1),1);
T_t=cell(1, length(FV1),1);
inv_kx = cell(1, length(FV1),1);
area = cell(1, length(FV1),1);
kx = zeros(12);
b=zeros(12*length(FV1), 1);
by=zeros(4*length(FV1), 1);
K = zeros(12*length(FV1), 3*length(FV1)+3*length(V1));

for i=1:length(FV1)
    v1 = [V1(FV1(i,1),1), V1(FV1(i,1),2),V1(FV1(i,1),3)]' ;
    v2 = [V1(FV1(i,2),1), V1(FV1(i,2),2), V1(FV1(i,2),3)]' ;
    v3 = [V1(FV1(i,3),1),V1(FV1(i,3),2),V1(FV1(i,3),3)]' ;
    v4 = 1/3*(v1+v2+v3) + cross(v2-v1, v3-v2)/sqrt(norm(cross(v2-v1, v3-v2)));
    V = [v1-v4, v2-v4, v3-v4];
    
    kx(1, 1:4) = [v1', 1];
    kx(2, 5:8) = [v1', 1];
    kx(3, 9:12) = [v1', 1];
    kx(4, 1:4) = [v2', 	1];
    kx(5, 5:8) = [v2', 1];
    kx(6, 9:12) = [v2', 1];
    kx(7, 1:4) = [v3', 1];
    kx(8, 5:8) = [v3', 1];
    kx(9, 9:12) = [v3',1];
    kx(10, 1:4) = [v4', 1];
    kx(11, 5:8) = [v4', 1];
    kx(12, 9:12) = [v4', 1];
    inv_kx{i} = inv(kx);
    
    u1 = [V2(FV2(i,1),1), V2(FV2(i,1),2),V2(FV2(i,1),3)]' ;
    u2 = [V2(FV2(i,2),1), V2(FV2(i,2),2), V2(FV2(i,2),3)]' ;
    u3 = [V2(FV2(i,3),1),V2(FV2(i,3),2),V2(FV2(i,3),3)]' ;

    u4 = 1/3*(u1+u2+u3) + cross(u2-u1, u3-u2)/sqrt(norm(cross(u2-u1, u3-u2)));
    U = [u1-u4, u2-u4, u3-u4];
    
    M = U/V;
    T{i} = u1-M*v1;
    
    [V,D,U] = svd(M);
    R{i} = V*U';
    S{i} = U*D*U';
    
      AB = v2-v1;
    AC = v3-v1;
    area{i} = norm(cross(AB,AC))/2;
end

for j=1:total_interpolations+1
    t=1/total_interpolations*(j-1);
    for i=1:length(FV1)
        S_t = (1-t)*eye(3) + t*S{i};
        T_t{i} = t*T{i};
        
        w = sqrt((1+R{i}(1,1)+R{i}(2,2)+R{i}(3,3)))/2; %quaternion stuff
        x = (R{i}(2,3) - R{i}(3,2))/(4*w);
        y = (R{i}(3,1) - R{i}(1,3))/(4*w);
        z = (R{i}(1,2)-R{i}(2,1))/(4*w);
        
        q =[w,x,y,z];
        angle = acos(dot(q0,q));
        
        q_t = 1/sin(angle)*(sin((1-t)*angle))*q0 + sin(t*angle)/sin(angle)*q;
        Rot_t = [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4), 2*q_t(4)*q_t(2)- 2*q_t(1)*q_t(3); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2, 2*q_t(3)*q_t(4) + 2*q_t(1)*q_t(2); 2*q_t(2)*q_t(4)+2*q_t(1)*q_t(3), 2*q_t(3)*q_t(4)-2*q_t(1)*q_t(2), 1- 2*q_t(2)^2 - 2*q_t(3)^2];
        M_t{i} = Rot_t*S_t;
        
        b(12*(i-1)+1:12*(i-1)+12) = area{i}*[M_t{i}(1,1), M_t{i}(1,2), M_t{i}(1,3), alpha*T_t{i}(1), M_t{i}(2,1), M_t{i}(2,2), M_t{i}(2,3), alpha*T_t{i}(2), M_t{i}(3,1), M_t{i}(3,2), M_t{i}(3,3), alpha*T_t{i}(3)]';
        
        for k=1:3
            for l=1:12
                K(12*(i-1)+l, 3*(FV1(i,k)-1)+1:3*(FV1(i,k)-1)+3) = area{i}*inv_kx{i}(l,3*(k-1)+1:3*(k-1)+3);
            end
        end
        
        K(12*(i-1)+1:12*(i-1)+12, 3*length(V1)+3*(i-1)+1) = area{i}*inv_kx{i}(1:12, 10);
        K(12*(i-1)+1:12*(i-1)+12, 3*length(V1)+3*(i-1)+2) = area{i}*inv_kx{i}(1:12, 11);
        K(12*(i-1)+1:12*(i-1)+12, 3*length(V1)+3*(i-1)+3) = area{i}*inv_kx{i}(1:12, 12);
    end
    
    V_new = (K'*K)\K'*b;
    V_intermediate = zeros(length(V1),3);
    for i=1:length(V1)
        V_intermediate(i,1) = V_new(3*(i-1)+1);
        V_intermediate(i,2) = V_new(3*(i-1)+2);
        V_intermediate (i,3) = V_new(3*(i-1)+3);
    end
    
    trimesh(FV1(:, 1:3), V_intermediate(:,1), V_intermediate(:,2), V_intermediate(:,3));
    xlabel('x')
    ylabel('y')
    zlabel('z')
    
    drawnow;
    
end