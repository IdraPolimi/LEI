clear all
close all
clc

a = 0.2;
joints2D = zeros(30,3);
errors2D = zeros(30,2);

joints3D = zeros(45,3);
errors3D = zeros(45,2);


SAMPLE_XML_PATH='SamplesConfig.xml';
KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);

I=mxNiPhoto(KinectHandles); I=permute(I,[3 2 1]);
h=imshow(I);
Pos= mxNiSkeleton(KinectHandles);
while(Pos(1)==0);
    mxNiUpdateContext(KinectHandles);
    I=mxNiPhoto(KinectHandles); I=permute(I,[3 2 1]);
    Pos= mxNiSkeleton(KinectHandles);
    set(h,'Cdata',I); drawnow;
end
hh=zeros(1,9);
zz = 1;
hold on
while(Pos(1)>0 && zz < 300)
    mxNiUpdateContext(KinectHandles);
    JTemp=mxNiPhoto(KinectHandles); JTemp=permute(JTemp,[3 2 1]);
    set(h,'Cdata',JTemp); drawnow;
    %J(zz,:,:,:)= JTemp;
    
    Pos = mxNiSkeleton(KinectHandles,1);
    PosJ(zz,:,: ) = Pos;
    
    tempPos2D = Pos(1:15,6:7)';
    tempPos3D = Pos(1:15,3:5)';
    joints2D = [joints2D(:,2:3), tempPos2D(:)];
    joints3D = [joints3D(:,2:3), tempPos3D(:)];
    
    for ii = 1:30
        [joints2D_(ii), tempEX2D] = brown(joints2D(ii,end),joints2D(ii,2), joints2D(ii,1), errors2D(ii,2), errors2D(ii,1),a);
        errors2D(ii,:) = [errors2D(ii,2),tempEX2D];
    end
    for ii = 1:45
        [joints3D_(ii), tempEX3D] = brown(joints3D(ii,end),joints3D(ii,2), joints3D(ii,1), errors3D(ii,2), errors3D(ii,1),a);
        errors3D(ii,:) = [errors3D(ii,2),tempEX3D];
    end
    
    %tempPos = Pos(1:15,6:7)';
    %joints = [joints(:,2:end) tempPos(:)];
    %ointsFilt = joints * window';
    
    if(hh(1)>0);
        for i=1:10   , delete(hh(i)); end
    end
    x2D_= joints2D_(1:2:end);
    xNonFilt = tempPos2D(1,1:15);
    y2D_= joints2D_(2:2:end);
    yNonFilt = tempPos2D(2,1:15);
    
    x3D_= joints3D_(1:3:end);
    y3D_= joints3D_(2:3:end);
    z3D_= joints3D_(3:3:end);
    %headX = x3D_(1);
    %headY = y3D_(1);
    %headZ = z3D_(1);
%   HEAD,NECK, LEFT SHOULDER, LEFT ELBOW, LEFT HAND,RIGHT SHOULDER
%        RIGHT ELBOW, RIGHT HAND, TORSO, LEFT HIP, LEFT KNEE, LEFT FOOT
%        RIGHT HIP, RIGHT KNEE, RIGHT FOOT
    
    
    hh(1)=plot(x2D_,y2D_,'rO');
    hh(2)=plot(x2D_([13 14 15]),y2D_([13 14 15]),'g');
    hh(3)=plot(x2D_([10 11 12]),y2D_([10 11 12]),'g');
    hh(4)=plot(x2D_([9 10]),y2D_([9 10]),'m');
    hh(5)=plot(x2D_([9 13]),y2D_([9 13]),'m');
    hh(6)=plot(x2D_([2 3 4 5]),y2D_([2 3 4 5]),'b');
    hh(7)=plot(x2D_([2 6 7 8]),y2D_([2 6 7 8]),'b');
    hh(8)=plot(x2D_([1 2]),y2D_([1 2]),'c');
    hh(9)=plot(x2D_([2 9]),y2D_([2 9]),'c');
    hh(10)=plot(xNonFilt,yNonFilt,'g*');
    drawnow
    pause(0.001)
    zz = zz+1;
end

mxNiDeleteContext(KinectHandles);