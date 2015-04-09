clear all;
close all;
addpath('rigidTransform')
K(1,1)= 164.255034407511;
K(1,2)= 0.0;
K(1,3)= 214.523999214172;
K(2,1)= 0.0;
K(2,2)= 164.255034407511;
K(2,3)= 119.433252334595;
K(3,1)= 0.0;
K(3,2)= 0.0;
K(3,3)= 1.0;

 
% im1 = im2double(rgb2gray(imread('/Users/zhangchen/Documents/courses/2015 Spring/Robot Autonomy/P2B/cmu_16662_p2/sensor_data/left000.jpg')));
% im2 = im2double(rgb2gray(imread('/Users/zhangchen/Documents/courses/2015 Spring/Robot Autonomy/P2B/cmu_16662_p2/sensor_data/right000.jpg')));
% 
% [worldPoints1,features1,xy1,sp1] = triangulateIm(im1,im2,K);
% 
% im1 = im2double(rgb2gray(imread('/Users/zhangchen/Documents/courses/2015 Spring/Robot Autonomy/P2B/cmu_16662_p2/sensor_data/left100.jpg')));
% im2 = im2double(rgb2gray(imread('/Users/zhangchen/Documents/courses/2015 Spring/Robot Autonomy/P2B/cmu_16662_p2/sensor_data/right100.jpg')));
% 
% [worldPoints2,features2,xy2,sp2] = triangulateIm(im1,im2,K);
% 
% [indexPairs] = matchFeatures(features1,features2,'MaxRatio',.4);
% 
% 
% wp1 = worldPoints1(1:3,indexPairs(:,1));
% wp2 = worldPoints2(1:3,indexPairs(:,2));
% bestT = estimateRigidTransformation(wp1',wp2',5000,0.01);
%THistory = struct('T',bestT);
% % matlabpool open 2
tstep = 1;

locList = [];
outfid = fopen('VO_OutputFile.txt', 'wt');
if outfid < 0
   error('file creation failed');
end
rtoutfid = fopen('RT_OutputFile.txt', 'wt');
if rtoutfid < 0
   error('file creation failed');
end


for i = 0:tstep:633
  
     im1 = im2double(rgb2gray(imread(sprintf('/home/ed/Documents/robonomy/hw2b/sensor_data/left%03d.jpg',i))));
     im2 = im2double(rgb2gray(imread(sprintf('/home/ed/Documents/robonomy/hw2b/sensor_data/right%03d.jpg',i))));

    im11 = im1;  
    [worldPoints1,features1,xy1,xy2,sp1] = triangulateIm(im1,im2,K);

    im1 = im2double(rgb2gray(imread(sprintf('/home/ed/Documents/robonomy/hw2b/sensor_data/left%03d.jpg',i+tstep))));
    im2 = im2double(rgb2gray(imread(sprintf('/home/ed/Documents/robonomy/hw2b/sensor_data/right%03d.jpg',i+tstep))));

    [worldPoints2,features2,xy12,xy22,sp2] = triangulateIm(im1,im2,K);

    [indexPairs] = matchFeatures(features1,features2,'MaxRatio',.5);

    wp1 = worldPoints1(1:3,indexPairs(:,1));
    wp2 = worldPoints2(1:3,indexPairs(:,2));
    
    uv1 = xy1(:,indexPairs(:,1));
    uv2 = xy2(:,indexPairs(:,1));
    
%     %make sure features are unique
    if(~size(locList))
        locList = features1(indexPairs(:,1),:);
        
    else
        matchedFeatures1 = [];
        matchedFeatures1 = features1(indexPairs(:,1),:);
        ips = matchFeatures(matchedFeatures1,locList,'MaxRatio',.5);
        %add unique ones to locList
        fullIDX = 1:size(matchedFeatures1,1);
        notIDX = ips(:,1);
        idx=setdiff(fullIDX,notIDX);
        uniqueFeatures = (matchedFeatures1(idx,:));
        locList = [locList; uniqueFeatures];
    end
%    %write to file with indexes
    ips = matchFeatures(features1(indexPairs(:,1),:),locList,'MaxRatio',.6);
%    
%    %x l uL uR v X Y Z
   for n = 1:size(ips,1)
%   
     fwrite(outfid, sprintf('%d %d %f %f %f %f %f %f\n',i,ips(n,2),uv1(1,n),uv2(1,n),uv1(2,n),wp1(1,n),wp1(2,n),wp1(3,n)));
    end 
% %     [bestT bestI] = ransacRigidT(wp1,wp2,xy1(:,indexPairs(:,1)),xy2(:,indexPairs(:,2)),sp1(:,indexPairs(:,1)),sp2(:,indexPairs(:,2)),K,2000,40);
%     %wp1size = size(wp1')
%     %wp2size = size(wp2')
%     bestT = estimateRigidTransformation(wp1',wp2',10000,0.1);
%     
%     fwrite(rtoutfid, sprintf('%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',i,bestT(1,1),bestT(1,2),bestT(1,3),bestT(1,4),bestT(2,1),bestT(2,2),bestT(2,3),bestT(2,4),bestT(3,1),bestT(3,2),bestT(3,3),bestT(3,4),bestT(4,1),bestT(4,2),bestT(4,3),bestT(4,4)));
%     
%     j = round((i+tstep)/tstep);%(i+5)/5;
%     THistory(j) = struct('T',bestT);
%     %VOfile{j} = sprintf('%d %d %f %f %f %f %f %f\n',i,ips(n,2),uv1(1,n),uv2(1,n),uv1(2,n),wp1(1,n),wp1(2,n),wp1(3,n));
%     %RTfile{j} =   sprintf('%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n',i,bestT(1,1),bestT(1,2),bestT(1,3),bestT(1,4),bestT(2,1),bestT(2,2),bestT(2,3),bestT(2,4),bestT(3,1),bestT(3,2),bestT(3,3),bestT(3,4),bestT(4,1),bestT(4,2),bestT(4,3),bestT(4,4));  
%     %j = j+1;
    fprintf('i = %d\n', i);
    
    
    
end



fclose(outfid);
fclose(rtoutfid);

% currentH = eye(4);
% points = [];
% %figure; hold on;
%  figure; hold on
% for i=1:length(THistory)
%      t = THistory(i).T(1:4,4);
%     
%     tDist = sqrt( t(1)^2 + t(2)^2 + t(3)^2);
%     if tDist > 0.4
% %         t =  t / (tDist / 0.4) 
%         newT = eye(4);
%     else
%         newT = [THistory(i).T(:,1:3) t];   
%     end
% 
% %     THistory(i).T
%     %pause(01);
%     %newT = [THistory(i).T(:,1:3) t];
%     currentH = newT*currentH;
%     points = [points currentH*[0;0;0;1]];
%     %currP  = currentH*[0;0;0;1];
%     %plot3(
%     plotTrajectory(currentH);
% end
% 
% points(1,:) = points(1,:)./points(4,:);
% points(2,:) = points(2,:)./points(4,:);
% points(3,:) = points(3,:)./points(4,:);
% 
% save('output.mat','THistory','points');
% 
% plot3(points(1,:),points(2,:),points(3,:),'b--o');
% xlabel('x - axis');
% ylabel('y - axis');
% zlabel('z - axis');
% title('3D Plot of Pose');


% matlabpool close;




%figure; hold on;
% for i=1:length(THistory)
%     t = THistory(i).T(1:4,4);
%     
%     tDist = sqrt( t(1)^2 + t(2)^2 + t(3)^2);
%     if tDist > 400
%         t =  t / (tDist / 400)
%         
%         newT = [THistory(i).T(:,1:3) t]; 
%         newT = eye(4);
%     else
%     newT = [THistory(i).T(:,1:3) t];    
%     end
% %     THistory(i).T
%     %pause(01);
%     %newT = [THistory(i).T(:,1:3) t];
%     currentH = newT*currentH;
%     points = [points currentH*[0;0;0;1]];
%     %currP  = currentH*[0;0;0;1];
%     %plot3(
% end
% 
% points(1,:) = points(1,:)./points(4,:);
% points(2,:) = points(2,:)./points(4,:);
% points(3,:) = points(3,:)./points(4,:);
% 
% save('output.mat','THistory','points');
% 
% figure; hold on;
% plot3(points(1,:),points(2,:),points(3,:),'b--o');
% xlabel('x');
% ylabel('y');
% zlabel('z');
% % min = 100;
% % max = 200;
% % axis([-1*min,1*max,-1*min,1*max,-1*min,1*max]);
% hold off;
% 
% % matlabpool close;
