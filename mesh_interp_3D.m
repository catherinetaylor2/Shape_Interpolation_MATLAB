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
t=0.5;
q0 = [1,0,0,0];
for i=1:length(FV1)
    v1 = [V1(FV1(i,1),1), V1(FV1(i,1),2),V1(FV1(i,3),3)]' ;
    v2 = [V1(FV1(i,2),1), V1(FV1(i,2),2), V1(FV1(i,2),3)]' ;
    v3 = [V1(FV1(i,3),1),V1(FV1(i,3),3),V1(FV1(i,3),3)]' ;
     if (sqrt(norm(cross(v2-v1, v3-v2)))~=0)
        a = cross(v2-v1, v3-v2)/sqrt(norm(cross(v2-v1, v3-v2)));
    else
        a=[0,0,0]';
    end
    v4 = 1/3*(v1+v2+v3) + a;
    V = [v1-v4, v2-v4, v3-v4];
    
    u1 = [V2(FV2(i,1),1), V2(FV2(i,1),2),V2(FV2(i,3),3)]' ;
    u2 = [V2(FV2(i,2),1), V2(FV2(i,2),2), V2(FV2(i,2),3)]' ;
    u3 = [V2(FV2(i,3),1),V2(FV2(i,3),3),V2(FV2(i,3),3)]' ;
    if (sqrt(norm(cross(u2-u1, u3-u2)))~=0)
        a = cross(u2-u1, u3-u2)/sqrt(norm(cross(u2-u1, u3-u2)));
    else
        a=[0,0,0]';
    end
    u4 = 1/3*(u1+u2+u3) + a;
    U = [u1-u4, u2-u4, u3-u4];
    
    M = V/U;
    T{i} = v1-M*u1;
    
    [V,D,U] = svd(M);
    R = V*U';
    S = U*D*U';
    S_t = (1-t)*eye(3) + t*S;
    T_t = t*T{i};
    
    w = sqrt(0.25*(1+R(1,1)+R(2,2)+R(3,3))); %quaternion stuff
    x = (R(2,3) - R(3,2))/(4*w);
    y = (R(3,1) - R(1,3))/(4*w);
    z = (R(1,2)-R(2,1))/(4*w);
    
    q =[w,x,y,z];
    angle = acos(dot(q0,q));
%     w_int = -q0(1)*w*dot(-q0(2:4),q(2:4));
%     q_int = q0(1)*q(2:4) + w*q(2:4) + cross(-q0(2:4),q(2:4));
%   qi = quatinterp(q0,q_int,t,'slerp')
%     
%     w_t = -w_int*q0(1)*dot(q_int,q0(2:4)); 
%     q_t  = w_int*q0(2:4) + q0(1)*q_int + cross(q_int,q0(2:4));

q_t = 1/sin(angle)*(sin((1-t)*angle))*q0 + sin(t*angle)/sin(angle)*q;

Rot_t = [1-2*q_t(3)^2 - 2*q_t(4)^2, 2*q_t(2)*q_t(3)+2*q_t(1)*q_t(4), 2*q_t(4)*q_t(2)- 2*q_t(1)*q_t(3); 2*q_t(2)*q_t(3)-2*q_t(1)*q_t(4), 1-2*q_t(2)^2 - 2*q_t(4)^2, 2*q_t(3)*q_t(4) + 2*q_t(1)*q_t(2); 2*q_t(2)*q_t(4)+2*q_t(1)*q_t(3), 2*q_t(3)*q_t(4)-2*q_t(1)*q_t(2), 1- 2*q_t(2)^2 - 2*q_t(4)^4];

M_t{i} = Rot_t*S_t;
    
end
