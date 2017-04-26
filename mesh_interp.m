%2D shape interp using paper

%input 2-meshes with 1-1 correspondance between triangles. 

close all;
clear;

obj1 = readObj('dino.obj'); %reads object file and stores vertices and faces.
FV1 = obj1.f.v;
V1 = obj1.v;

a = V1(:,1); %deals with mesh data.
V1(:,1) = -V1(:,3);
V1(:,2) = -a;  
V1(:,3) = zeros(length(V1),1);

obj2 = readObj('keyframe1.obj');
FV2 = obj2.f.v;
V2 = obj2.v;
V2(:,2) = -V2(:,2);

figure
% subplot(2,1,1)
trimesh(FV1, V1(:,1), V1(:,2));
hold on
% subplot(2,1,2);
% trimesh(FV2, V2(:,1), V2(:,2));

P=cell(1,length(FV1),1);
Q=cell(1, length(FV2),1);
Ai=cell(1,length(FV1),1);
A_full = zeros(4*length(FV1)+2, 2*length(V1));
b=zeros(4*length(FV1)+2,1);
Px = zeros(6);
Qx = zeros(6,1);
interpolations = 100;
t=0.5;
for i=1:length(FV1) %calculate A for each triangle. 
    P{i} = [V1(FV1(i,1),1), V1(FV1(i,1),2); V1(FV1(i,2),1), V1(FV1(i,2),2); V1(FV1(i,3),1), V1(FV1(i,3),2)]; %build each source triangle
    Q{i} = [V2(FV2(i,1),1), V2(FV2(i,1),2); V2(FV2(i,2),1), V2(FV2(i,2),2); V2(FV2(i,3),1), V2(FV2(i,3),2)]; %build each target triangle
   
%     figure
   plot( P{i}(:,1), P{i}(:,2), 'o')
   hold on
   plot( Q{i}(:,1), Q{i}(:,2), 'ro')
   
    
    for j =1:3
        Px(2*(j-1)+1,:)= [P{i}(j,1), P{i}(j,2), 1, 0,0,0];
        Px(2*(j-1)+2,:) = [0,0,0, P{i}(j,1), P{i}(j,2), 1];
        Qx(2*(j-1)+1) = Q{i}(j,1);
        Qx(2*(j-1)+2) = Q{i}(j,2);
    end
    inv_px = inv(Px);
    Al = Px\Qx;
    Ai{i} = [Al(1), Al(2); Al(4), Al(5)]; %find ideal affine transformation matrix for each triangle.     
    [V,D,U] = svd(Ai{i}); %decompose using single value decomposition.
    Ut=U';
    S = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    Rt = [(R(1,1)-1)*t+1, (R(1,2))*t; (R(2,1))*t, (R(2,2)-1)*t+1];
    At = Rt*((1-t)*eye(2) +  t*(i-1)*S);
    b(4*(i-1)+1:4*(i-1)+4)=[At(1,1), At(1,2), At(2,1), At(2,2)]';
    
    for k=1:3
        A_full(4*(i-1)+1, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px(1,2*(k-1)+1:2*(k-1)+2);
        A_full(4*(i-1)+2, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px(2,2*(k-1)+1:2*(k-1)+2);
        A_full(4*(i-1)+3, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px(4,2*(k-1)+1:2*(k-1)+2);
        A_full(4*(i-1)+4, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px(5,2*(k-1)+1:2*(k-1)+2);
    end
end
b(4*length(FV1)+1)= (1-t)*V1(1)+t*V2(1);
b(4*length(FV1)+2)= (1-t)*V1(2)+t*V2(2);
A(4*length(FV1)+1,1) = 1;
A(4*length(FV1)+2,2)=1

A = A_full;

V_new = (A'*A)'\A'*b;
V_1=zeros(length(V1),2);
for i =1:length(V_new)
    if (mod(i,2)==0)
        V_1(i/2, 2) = V_new(i);
    else
        V_1((i-1)/2+1,1) = V_new(i);
    end
end


figure
trimesh(FV1, V_1(:,1), V_1(:,2));