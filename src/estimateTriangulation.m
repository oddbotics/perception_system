imLeft = im2double(rgb2gray(imread('images/left012.jpg')));
imRight = im2double(rgb2gray(imread('images/right012.jpg')));
rect = [540 295 683 468];
load('template.mat');
H=vision.TemplateMatcher;

close all;
pause(.001);



imshow(imLeft);
hold on;
%find template
LOC = step(H,imLeft,template);

height = rect(4) - rect(2);
width = rect(3) - rect(1);

plot(LOC(1)-width/2,LOC(2)-height/2,'bO');
rectangle('Position',[LOC(1) - width/2,LOC(2) - height/2,width,height],'EdgeColor','blue');

hold off;
pause(.7);

close all;

R = stereoParams.RotationOfCamera2;
t = stereoParams.TranslationOfCamera2';
Rt = [R t];
[worldPoints,matchedFeatures,pts1,pts2,sp1] = triangulateIm(imLeft,imRight, stereoParams.CameraParameters1.IntrinsicMatrix, stereoParams.CameraParameters2.IntrinsicMatrix,Rt);
onBoxPts = [];
for i=1:length(pts1)
    if(pts1(1,i) > rect(1) && pts1(1,i) < rect(3) && pts1(2,i) > rect(2) && pts1(2,i) < rect(4))
    worldPoints(:,i)
    onBoxPts = [onBoxPts i];
    end
end
imshow(imLeft);
hold on;
plot(pts1(1,onBoxPts),pts1(2,onBoxPts),'rO')
hold off;

figure;
imshow(imRight);
hold on;
plot(pts2(1,onBoxPts),pts2(2,onBoxPts),'bO')
hold off;