 image_dir = '/home/ed/Dropbox/GitHub/catkin_ws/src/perception_system/src/images/calibration_images/20APR';
leftImages = imageSet(fullfile(image_dir,'left'));
rightImages = imageSet(fullfile(image_dir,'right'));
images1 = leftImages.ImageLocation;
images2 = rightImages.ImageLocation;
[imagePoints,boardSize,pairsUsed] = detectCheckerboardPoints(images1,images2);
squareSize = 22.5;
worldPoints = generateCheckerboardPoints(boardSize,squareSize);
stereoParams = estimateCameraParameters(imagePoints,worldPoints);
save('/home/ed/Dropbox/GitHub/catkin_ws/src/perception_system/src/images/calibration_images/20APR/stereoParams.mat','stereoParams');
save('/home/ed/Dropbox/GitHub/catkin_ws/src/perception_system/src/images/calibration_images/20APR/calibParams.mat');
K1 = stereoParams.CameraParameters1.IntrinsicMatrix;
K2 = stereoParams.CameraParameters2.IntrinsicMatrix;
M1 = [eye(3) zeros(3,1)];
R2 = stereoParams.RotationOfCamera2;
t2 = stereoParams.TranslationOfCamera2;
M2 = [R2 t2'];
P1 = K1*M1;
P2 = K2*M2;


matlab2opencv(P1,'20APRprojmats.yml');
matlab2opencv(P2,'20APRprojmats.yml','a');



