% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Carries out as-rigid-as-possible shape interpolation using 2D meshes.
%Requires two meshes with 1-1 correspondance between triangles. 

close all;
clear;

obj1 = readObj('keyframe3.obj'); %reads object file and stores vertices and faces.
FV1 = obj1.f.v;
V1 = obj1.v;

% a = V1(:,1); %deals with mesh data.
% V1(:,1) = -V1(:,3);
% V1(:,2) = -a;  
% V1(:,3) = zeros(length(V1),1);
V1(:,2) = -V1(:,2);

obj2 = readObj('keyframe4.obj');
FV2 = obj2.f.v;
V2 = obj2.v;
V2(:,2) = -V2(:,2);

figure
subplot(1,2,1)
trimesh(FV1, V1(:,1), V1(:,2));
hold on
subplot(1,2,2);
trimesh(FV2, V2(:,1), V2(:,2));
figure
% P=cell(1,length(FV1),1);
% Q=cell(1, length(FV2),1);
Ai=cell(1,length(FV1),1);
inv_px=cell(1,length(FV1),1);
A = zeros(4*length(FV1)+2, 2*length(V1));
b=zeros(4*length(FV1)+2,1);
Px = zeros(6);
Qx = zeros(6,1);
interpolations = 100;
% for l=1:interpolations+1
t=1/interpolations*50;
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
for i =1:length(FV1)
    [V,D,U] = svd(Ai{i}); %decompose using single value decomposition.
    Ut=U';
    S = U*D*U'; %symmetric matrix
    R = V*U'; %rotation matrix.
    
    Rt = [(R(1,1)-1)*t+1, (R(1,2))*t; (R(2,1))*t, (R(2,2)-1)*t+1];
    At = Rt*((1-t)*eye(2) +  t*S);
    b(4*(i-1)+1:4*(i-1)+4)=[At(1,1), At(1,2), At(2,1), At(2,2)]';
    
    for k=1:3
        A(4*(i-1)+1, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{i}(1,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+2, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{i}(2,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+3, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{i}(4,2*(k-1)+1:2*(k-1)+2);
        A(4*(i-1)+4, 2*(FV1(i,k)-1)+1:2*(FV1(i,k)-1)+2) = inv_px{i}(5,2*(k-1)+1:2*(k-1)+2);
    end
end
b(4*length(FV1)+1)= (1-t)*V1(1,1)+t*V2(1,1);
b(4*length(FV1)+2)= (1-t)*V1(1,2)+t*V2(1,2);
A(4*length(FV1)+1,1) = 1;
A(4*length(FV1)+2,2)=1;

V_new = (A'*A)'\A'*b;
V_1=zeros(length(V1),2);
for i =1:length(V_new)
    if (mod(i,2)==0)
        V_1(i/2, 2) = V_new(i);
    else
        V_1((i-1)/2+1,1) = V_new(i);
    end
end

trimesh(FV1, V_1(:,1), V_1(:,2));
axis([-20 20 -15 15]);
drawnow;

% end