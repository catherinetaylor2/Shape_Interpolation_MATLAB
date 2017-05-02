%as rigid as poss 3D

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
xlabel('x')
ylabel('y')
zlabel('z')
axis([-1, 1, 1, 3,-1,1]);

M_t=cell(1, length(FV1), 1);
T=cell(1, length(FV1),1);
T_t=cell(1, length(FV1),1);
inv_kx = cell(1, length(FV1),1);
area = cell(1, length(FV1),1);
t=0.5;
q0 = [1,0,0,0];
alpha = 1;
kx = zeros(12);
for i=1:length(FV1)
    v1 = [V1(FV1(i,1),1), V1(FV1(i,1),2),V1(FV1(i,1),3)]' ;
    v2 = [V1(FV1(i,2),1), V1(FV1(i,2),2), V1(FV1(i,2),3)]' ;
    v3 = [V1(FV1(i,3),1),V1(FV1(i,3),2),V1(FV1(i,3),3)]' ;
    v4 = 1/3*(v1+v2+v3) + cross(v2-v1, v3-v2)/sqrt(norm(cross(v2-v1, v3-v2)));
    V = [v1-v4, v2-v4, v3-v4];
    
    kx(1, 1:4) = [v1', alpha];
    kx(2, 5:8) = [v1', alpha];
    kx(3, 9:12) = [v1', alpha];
    kx(4, 1:4) = [v2', alpha];
    kx(5, 5:8) = [v2', alpha];
    kx(6, 9:12) = [v2', alpha];
    kx(7, 1:4) = [v3', alpha];
    kx(8, 5:8) = [v3', alpha];
    kx(9, 9:12) = [v3', alpha];
      kx(10, 1:4) = [v4', alpha];
    kx(11, 5:8) = [v4', alpha];
    kx(12, 9:12) = [v4', alpha];
%     kx= [v1', alpha, 0,0,0,0,0,0,0,0;0,0,0,0; v1', alpha, 0,0,0,0;0,0,0,0,0,0,0,0,v1', alpha; v2', alpha,0,0,0,0,0,0,0,0; 0,0,0,0,v2', alpha,0,0,0,0;0,0,0,0,0,0,0,0,v2', alpha; v3', alpha; v4', alpha];
    inv_kx{i} = inv(kx); 
    
    u1 = [V2(FV2(i,1),1), V2(FV2(i,1),2),V2(FV2(i,1),3)]' ;
    u2 = [V2(FV2(i,2),1), V2(FV2(i,2),2), V2(FV2(i,2),3)]' ;
    u3 = [V2(FV2(i,3),1),V2(FV2(i,3),2),V2(FV2(i,3),3)]' ;

    u4 = 1/3*(u1+u2+u3) + cross(u2-u1, u3-u2)/sqrt(norm(cross(u2-u1, u3-u2)));
    U = [u1-u4, u2-u4, u3-u4];
    
    M = U/V;
    T{i} = u1-M*v1;
    
    [V,D,U] = svd(M);
    R = V*U';
    S = U*D*U';
    S_t = (1-t)*eye(3) + t*S;
    T_t{i} = t*T{i};
    
    w = sqrt((1+R(1,1)+R(2,2)+R(3,3)))/2; %quaternion stuff
    x = (R(2,3) - R(3,2))/(4*w);
    y = (R(3,1) - R(1,3))/(4*w);
    z = (R(1,2)-R(2,1))/(4*w);
    
    q =[w,x,y,z];
    angle = acos(dot(q0,q));

    q_t = 1/sin(angle)*(sin((1-t)*angle))*q0 + sin(t*angle)/sin(angle)*q;
    Rot_t = [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4), 2*q_t(4)*q_t(2)- 2*q_t(1)*q_t(3); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2, 2*q_t(3)*q_t(4) + 2*q_t(1)*q_t(2); 2*q_t(2)*q_t(4)+2*q_t(1)*q_t(3), 2*q_t(3)*q_t(4)-2*q_t(1)*q_t(2), 1- 2*q_t(2)^2 - 2*q_t(3)^2];
    M_t{i} = Rot_t*S_t;
    
    AB = v2-v1;
    AC = v3-v1;
    area{i} = 1; %norm(cross(AB,AC))/2;
end

b=zeros(12*length(FV1), 1);
by=zeros(4*length(FV1), 1);

K = zeros(12*length(FV1), 3*length(FV1)+3*length(V1));

for i=1:length(FV1)
    
    b(12*(i-1)+1:12*(i-1)+12) = area{i}*[M_t{i}(1,1), M_t{i}(1,2), M_t{i}(1,3), alpha*T_t{i}(1), M_t{i}(2,1), M_t{i}(2,2), M_t{i}(2,3), alpha*T_t{i}(2), M_t{i}(3,1), M_t{i}(3,2), M_t{i}(3,3), T_t{i}(3)]';
    
    for k=1:3
        for j=1:12
        K(12*(i-1)+j, 3*(FV1(i,k)-1)+1:3*(FV1(i,k)-1)+3) = area{i}*inv_kx{i}(j,3*(k-1)+1:3*(k-1)+3);
        end
    end
    
    K(12*(i-1)+1:12*(i-1)+12, 3*length(V1)+3*(i-1)+1) = area{i}*inv_kx{i}(1:12, 10);
    K(12*(i-1)+1:12*(i-1)+12, 3*length(V1)+3*(i-1)+2) = area{i}*inv_kx{i}(1:12, 11);
    K(12*(i-1)+1:12*(i-1)+12, 3*length(V1)+3*(i-1)+3) = area{i}*inv_kx{i}(1:12, 12);
end

V_new = (K'*K)\K'*b;

for i=1:length(V1)
   V_intermediate(i,1) = V_new(3*(i-1)+1);
   V_intermediate(i,2) = V_new(3*(i-1)+2);
   V_intermediate (i,3) = V_new(3*(i-1)+3);
end

figure
trimesh(FV1(:, 1:3), V_intermediate(:,1), V_intermediate(:,2), V_intermediate(:,3)); 
xlabel('x')
ylabel('y')
zlabel('z')
axis([-1, 1, 1, 3,-1,1]);

