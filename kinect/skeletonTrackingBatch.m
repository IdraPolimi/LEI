%%%% HEAD,NECK, LEFT SHOULDER, LEFT ELBOW, LEFT HAND,RIGHT SHOULDER
%        RIGHT ELBOW, RIGHT HAND, TORSO, LEFT HIP, LEFT KNEE, LEFT FOOT
%        RIGHT HIP, RIGHT KNEE, RIGHT FOOT


a1 = 0.2;
a2 = 0.1;

%loadRobotParam
%[clientID,vrep] = connectToRobot(IPADDRESS, PORT, WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS);
%vrep.simxSynchronous(clientID,false);
%for jj = 1:3
%    [~, jointsHandle(jj)] = vrep.simxGetObjectHandle(clientID, cell2mat(jointsName(jj)), vrep.simx_opmode_oneshot_wait);
%    tempLimit = jointsLimits(strcmp({jointsLimits.name},cell2mat(jointsName(jj))));
%    upperLimits(jj) = tempLimit.upperLimit;
%    lowerLimits(jj) = tempLimit.lowerLimit;
%end
%if(~vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot))
%    disp('Error in starting simulation');
%    return
%end





joints3D = zeros(45,3);

hh=zeros(1,9);
zz = 1;
figure
hold on
%set(gca,'YDir','reverse');
set(gca,'XDir','reverse');
%set(gca,'ZDir','reverse');
axis([-1000 1500 -1000 1500 1100 2900])

%%%
Pos1 = squeeze(PosJ(1,:,:));
Pos2 = squeeze(PosJ(2,:,:));

% xList(1) = Pos1(4,3);
% yList(1) = Pos1(4,4);
% zList(1) = Pos1(4,5);
% xList(2) = Pos2(4,3);
% yList(2) = Pos2(4,4);
% zList(2) = Pos2(4,5);

tempPos3D1 = Pos1(1:15,3:5)';
tempPos3D2 = Pos2(1:15,3:5)';
joints3D = [joints3D(:,3),tempPos3D1(:),tempPos3D2(:)];
joints3D_ = [joints3D(:,3),tempPos3D1(:),tempPos3D1(:)];


%joints3D__ = tempPos3D1(:)';
% xList_(1:2) = joints3D_(10,2:end);
% yList_(1:2) = joints3D_(11,2:end);
% zList_(1:2) = joints3D_(12,2:end);
% xList__(1) = joints3D__(10);
% yList__(1) = joints3D__(11);
% zList__(1) = joints3D__(12);
% xList__(2) = joints3D__(10);
% yList__(2) = joints3D__(11);
% zList__(2) = joints3D__(12);

errors3D = [tempPos3D1(:)-tempPos3D1(:), tempPos3D2(:)-tempPos3D1(:)];
errors3D_ = errors3D;

for kk = 3:size(PosJ,1)
    Pos = squeeze(PosJ(kk,:,:));
    tempPos3D = Pos(1:15,3:5)';
    joints3D = [joints3D(:,2:3), tempPos3D(:)];
    
    tempVec = zeros(45,1);
    for ii = 1:45
        [tempJoints3D_, tempEX3D] = brown(joints3D(ii,end),joints3D(ii,2), joints3D(ii,1), errors3D(ii,2), errors3D(ii,1),a1);
        tempVec(ii) = tempJoints3D_;
        errors3D(ii,:) = [errors3D(ii,2),tempEX3D];
    end
    joints3D_ = [joints3D_(:,2:3), tempVec];
    
    for ii = 1:45
        [joints3D__(ii), tempEX3D_] = brown(joints3D_(ii,end),joints3D_(ii,2), joints3D_(ii,1), errors3D_(ii,2), errors3D_(ii,1),a2);
        errors3D_(ii,:) = [errors3D_(ii,2),tempEX3D_];
    end
    if(hh(1)>0);
        for i=1:9
            delete(hh(i));
        end
    end
    
    %    xNonFilt= joints3D(1:3:end);
    %    yNonFilt= joints3D(2:3:end);
    %    zNonFilt= joints3D(3:3:end);
    
    x3D_= joints3D__(1:3:end);
    y3D_= joints3D__(2:3:end);
    z3D_= joints3D__(3:3:end);
    
    %     xList(kk) = Pos(4,3);
    %     yList(kk) = Pos(4,4);
    %     zList(kk) = Pos(4,5);
    %     xList_(kk) = joints3D_(10,end);
    %     yList_(kk) = joints3D_(11,end);
    %     zList_(kk) = joints3D_(12,end);
    %     xList__(kk) = joints3D__(10);
    %     yList__(kk) = joints3D__(11);
    %     zList__(kk) = joints3D__(12);
    hh(1)=plot3(x3D_,y3D_,z3D_,'rO');
    hh(2)=plot3(x3D_([13 14 15]),y3D_([13 14 15]),z3D_([13 14 15]),'g');%leftLeg
    hh(3)=plot3(x3D_([10 11 12]),y3D_([10 11 12]),z3D_([10 11 12]),'g');%rightLeg
    hh(4)=plot3(x3D_([9 10]),y3D_([9 10]),z3D_([9 10]),'m');%upperLeftLeg
    hh(5)=plot3(x3D_([9 13]),y3D_([9 13]),z3D_([9 13]),'m');%upperRightLeg
    hh(6)=plot3(x3D_([2 3 4 5]),y3D_([2 3 4 5]),z3D_([2 3 4 5]),'b');%leftArm
    hh(7)=plot3(x3D_([2 6 7 8]),y3D_([2 6 7 8]),z3D_([2 6 7 8]),'b');%rightArm
    hh(8)=plot3(x3D_([1 2]),y3D_([1 2]),z3D_([1 2]),'c');%neck
    hh(9)=plot3(x3D_([2 9]),y3D_([2 9]),z3D_([2 9]),'c--');%torso
    %     hh(10)=plot3(xNonFilt,yNonFilt,zNonFilt,'g*');
    drawnow
    
    %neck = [x3D_(1),y3D_(1), z3D_(1)] - [x3D_(2), y3D_(2), z3D_(2)];
    %torso = [x3D_(9),y3D_(9), z3D_(9)] - [x3D_(2),y3D_(2), z3D_(2)];
    
    leftShoulder = [x3D_(2),y3D_(2), z3D_(2)] - [x3D_(3),y3D_(3), z3D_(3)];
    leftUpperArm = [x3D_(4),y3D_(4), z3D_(4)] - [x3D_(3),y3D_(3), z3D_(3)];
    leftLowerArm = [x3D_(5),y3D_(5), z3D_(5)] - [x3D_(4),y3D_(4), z3D_(4)];
    
    %rightShoulder = [x3D_(2),y3D_(2), z3D_(2)] - [x3D_(6),y3D_(6), z3D_(6)];
    %rightUpperArm = [x3D_(6),y3D_(6), z3D_(6)] - [x3D_(7),y3D_(7), z3D_(7)];
    %rightLowerArm = [x3D_(7),y3D_(7), z3D_(7)] - [x3D_(8),y3D_(8), z3D_(8)];
    
    %%compute shoulderPitch and shoulderRoll
    upperArmProj = leftUpperArm;
    upperArmProj(3) = 0;
    %y(1,zz) = computeAngle(leftUpperArm, upperArmProj); %shoulderPitch
    upperArmProj(3) = leftUpperArm(3);
    upperArmProj(1) = 0;
    y(2,zz) = computeAngle(leftUpperArm, upperArmProj); %shoulderRoll
    
    %%compute elbowRoll
    %y(3,zz) = computeAngle(leftUpperArm, leftLowerArm); %leRoll OK
    
    %%compute elbowYaw
    normalToPlane = cross(leftUpperArm,leftLowerArm);
    upperArmProJ = leftUpperArm;
    upperArmProj(1) = 0;
    normalToVerticalPlane = cross(leftUpperArm,upperArmProj);
    %y(4,zz) = computeAngle(normalToPlane, normalToVerticalPlane); %elbow Yaw
    
   
    y(2,zz)*180/pi
    %%Old angle.
    %y(1,zz) = pi/2 - computeAngle(leftUpperArm, torso); %lsPitch
    %y(2,zz) = computeAngle(leftUpperArm, leftShoulder)-pi/2; %lsRoll  
    %y(3,zz) = computeAngle(leftUpperArm, leftLowerArm); %leRoll OOK
     
    
    
    %moveRobot(jointsHandle, y(:,zz), upperLimits, lowerLimits, vrep, clientID, 0.01);
   
    pause(0.4)
    
    zz = zz+1;
end

%vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
%vrep.simxFinish(clientID);