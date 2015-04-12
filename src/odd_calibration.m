image_dir = '/home/ed/Dropbox/GitHub/catkin_ws/src/perception_system/src/images/calibration_images/12APR';
leftImages = imageSet(fullfile(image_dir,'left'));
rightImages = imageSet(fullfile(image_dir,'right'));
images1 = leftImages.ImageLocation;
images2 = rightImages.ImageLocation;
[imagePoints,boardSize,pairsUsed] = detectCheckerboardPoints(images1,images2);
squareSize = 22.5;
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

