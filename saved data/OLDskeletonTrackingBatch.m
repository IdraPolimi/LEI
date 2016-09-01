%%%% HEAD,NECK, LEFT SHOULDER, LEFT ELBOW, LEFT HAND,RIGHT SHOULDER
%        RIGHT ELBOW, RIGHT HAND, TORSO, LEFT HIP, LEFT KNEE, LEFT FOOT
%        RIGHT HIP, RIGHT KNEE, RIGHT FOOT


a = 0.2;

loadRobotParam
[clientID,vrep] = connectToRobot(IPADDRESS, PORT, WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS);
vrep.simxSynchronous(clientID,false);
for jj = 1:3
    [~, jointsHandle(jj)] = vrep.simxGetObjectHandle(clientID, cell2mat(jointsName(jj)), vrep.simx_opmode_oneshot_wait);
    tempLimit = jointsLimits(strcmp({jointsLimits.name},cell2mat(jointsName(jj))));
    upperLimits(jj) = tempLimit.upperLimit;
    lowerLimits(jj) = tempLimit.lowerLimit;
end
if(~vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot))
    disp('Error in starting simulation');
    return
end

joints2D = zeros(30,3);
errors2D = zeros(30,2);

joints3D = zeros(45,3);
errors3D = zeros(45,2);

%hh=zeros(1,9);

zz = 1;
%figure
%hold on
%set(gca,'YDir','reverse');
%set(gca,'XDir','reverse');
%set(gca,'ZDir','reverse');
%axis([-1000 1500 -1000 1500 1500 2900])
for kk = 1:size(PosJ,1)
    Pos = squeeze(PosJ(zz,:,:));
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
    
    %     if(hh(1)>0);
    %         for i=1:9
    %             delete(hh(i));
    %         end
    %     end
    x2D_= joints2D_(1:2:end);
    y2D_= joints2D_(2:2:end);
    
    x3D_= joints3D_(1:3:end);
    y3D_= joints3D_(2:3:end);
    z3D_= joints3D_(3:3:end);
    
    %     hh(1)=plot(x2D_,y2D_,'rO');
    %     hh(2)=plot(x2D_([13 14 15]),y2D_([13 14 15]),'g');%leftLeg
    %     hh(3)=plot(x2D_([10 11 12]),y2D_([10 11 12]),'g--');%rightLeg
    %     hh(4)=plot(x2D_([9 10]),y2D_([9 10]),'m');%upperLeftLeg
    %     hh(5)=plot(x2D_([9 13]),y2D_([9 13]),'m');%upperRightLeg
    %     hh(6)=plot(x2D_([2 3 4 5]),y2D_([2 3 4 5]),'b');%rightArm
    %     hh(7)=plot(x2D_([2 6 7 8]),y2D_([2 6 7 8]),'b');%lefttArm
    %     hh(8)=plot(x2D_([1 2]),y2D_([1 2]),'c');%neck
    %     hh(9)=plot(x2D_([2 9]),y2D_([2 9]),'c');%torso
    
    %     hh(1)=plot3(x3D_,y3D_,z3D_,'rO');
    %     hh(2)=plot3(x3D_([13 14 15]),y3D_([13 14 15]),z3D_([13 14 15]),'g');%leftLeg
    %     hh(3)=plot3(x3D_([10 11 12]),y3D_([10 11 12]),z3D_([10 11 12]),'g');%rightLeg
    %     hh(4)=plot3(x3D_([9 10]),y3D_([9 10]),z3D_([9 10]),'m');%upperLeftLeg
    %     hh(5)=plot3(x3D_([9 13]),y3D_([9 13]),z3D_([9 13]),'m');%upperRightLeg
    %     hh(6)=plot3(x3D_([2 3 4 5]),y3D_([2 3 4 5]),z3D_([2 3 4 5]),'b');%leftArm
    %     hh(7)=plot3(x3D_([2 6 7 8]),y3D_([2 6 7 8]),z3D_([2 6 7 8]),'b');%rightArm
    %     hh(8)=plot3(x3D_([1 2]),y3D_([1 2]),z3D_([1 2]),'c');%neck
    %     hh(9)=plot3(x3D_([2 9]),y3D_([2 9]),z3D_([2 9]),'c--');%torso
    %     %hh(10)=plot(xNonFilt,yNonFilt,'g*');
    %     drawnow
    
    neck = [x3D_(1),y3D_(1), z3D_(1)] - [x3D_(2), y3D_(2), z3D_(2)];
    torso = [x3D_(9),y3D_(9), z3D_(9)] - [x3D_(2),y3D_(2), z3D_(2)];
        
    leftShoulder = [x3D_(2),y3D_(2), z3D_(2)] - [x3D_(3),y3D_(3), z3D_(3)];
    leftUpperArm = [x3D_(4),y3D_(4), z3D_(4)] - [x3D_(3),y3D_(3), z3D_(3)];
    leftLowerArm = [x3D_(5),y3D_(5), z3D_(5)] - [x3D_(4),y3D_(4), z3D_(4)];
    
    rightShoulder = [x3D_(2),y3D_(2), z3D_(2)] - [x3D_(6),y3D_(6), z3D_(6)];
    rightUpperArm = [x3D_(6),y3D_(6), z3D_(6)] - [x3D_(7),y3D_(7), z3D_(7)];
    rightLowerArm = [x3D_(7),y3D_(7), z3D_(7)] - [x3D_(8),y3D_(8), z3D_(8)];
  
    y(1) = pi/2 - computeAngle(leftUpperArm, torso); %lsPitch
    y(2) = computeAngle(leftUpperArm, leftShoulder)-pi/2; %lsRoll
    y(3) = computeAngle(leftUpperArm, leftLowerArm); %leRoll
    
    
    moveRobot(jointsHandle, y, upperLimits, lowerLimits, vrep, clientID, 0.01);
    
    pause(0.01)
    zz = zz+1;
end

vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
vrep.simxFinish(clientID);

