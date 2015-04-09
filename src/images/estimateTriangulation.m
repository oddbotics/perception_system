imLeft = double(rgb2gray(imread('left012.jpg')));
imRight = double(rgb2gray(imread('right012.jpg')));

[worldPoints,matchedFeatures,pts1,pts2,sp1] = triangulateIm(imLeft,imRight, stereoParams.CameraParameters1.IntrinsicMatrix, stereoParams.CameraParameters2.IntrinsicMatrix);
