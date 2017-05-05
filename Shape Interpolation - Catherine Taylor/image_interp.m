% ----------Computer Animation and Games 2: Coursework 2-------------------
% ----------------- Catherine Taylor : s169394549 -------------------------

%Linear interpolation between two similar images.

clear;
close all;

I1 = imread('beach_bw.bmp');
I2 = imread('beach.bmp');

interpolations = 100;
for i=1:interpolations
   I_interp = (1-1/interpolations*(i-1))*I1 + 1/interpolations*(i-1)*I2;
   imshow(I_interp);
   drawnow;
end