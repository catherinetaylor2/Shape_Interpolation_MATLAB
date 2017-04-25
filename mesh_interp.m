%mesh interp

%for one triangle

P = [1, 0; 1, 1; 2, 0];
Q = [0,0; 0,1; 1,0];

Px = zeros(6);
Qx = zeros(6,1);
for i =1:3
    Px(2*(i-1)+1,:)= [P(i,1), P(i,2), 1, 0,0,0];
    Px(2*(i-1)+2,:) = [0,0,0, P(i,1), P(i,2), 1];
    Qx(2*(i-1)+1) = Q(i,1);
    Qx(2*(i-1)+2) = Q(i,2);
end

Al = Px\Qx;
A = [Al(1), Al(2); Al(4), Al(5)];

[V,D,U] = svd(A);

S = U*D*U';
R = V*U';
l = R*S;

interpolations=100;
for i=1: interpolations
   At = R*((1-1/interpolations*(i-1))*eye(2) +  1/interpolations*(i-1)*S);
   V = [(At*P(1,:)')'; (At*P(2,:)')'; (At*P(3,:)')'];
   plot(V(:,1), V(:,2), 'o');
   axis([0,2.5,0,1.5]);
   drawnow;
end