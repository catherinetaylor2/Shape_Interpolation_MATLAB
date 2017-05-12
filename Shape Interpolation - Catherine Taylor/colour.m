C = imread('dino_texture.bmp');
for k=1 :length(FV1)
    index1 = FV1(k,1);
    index2 = FV1(k,2);
    index3 = FV1(k,3);
    c=C(1:3,k);
    x = [V1(index1,1), V1(index2,1), V1(index3,1)]';
    y = [V1(index1,2), V1(index2,2), V1(index3,2)]';
    z = [V1(index1,3), V1(index2,3), V1(index3,3)]';
   fill3(x,y,z, c); 
   hold all
end