%linear image interp
clear;
close all;

% I1 = imread('beach_bw.bmp');
% I2 = imread('beach.bmp');

I1 = imread('a.png');
I2 = imread('b.png');


interpolations = 100;
for i=1:interpolations
   I_interp = (1-1/interpolations*(i-1))*I1 + 1/interpolations*(i-1)*I2;
   imshow(I_interp);
   drawnow;
end