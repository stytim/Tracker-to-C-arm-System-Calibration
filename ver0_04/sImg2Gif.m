function sImg2GIF(filename, figNo)
% sImg2GIF - saves a figure into a GIF image
%
%	   sImg2GIF(filename, figNo)
%  
%      filename - string with a filename
%      figNo - figure number, default is current figure (optional)


if nargin == 2, figure(figNo); end;

[IMG,map] = capture;
f = figure;
imshow(IMG,map); truesize;
disp('Select the crop region');
[B,rect] = imcrop;
gifwrite(B, map, filename ); 
close(f);

return
