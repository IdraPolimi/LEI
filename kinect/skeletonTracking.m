
%% parameters
a1 = 0.2;
a2 = 0.1;
nJoints = 3;
useRobot = false;
useKinect = false;
movementData = 'rawDataElbowYaw';

%% Trying to connect to kinect. If batch, load movement data
if(useKinect)
    disp('Starting Kinect...');
    SAMPLE_XML_PATH='SamplesConfig.xml';
    KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);
    disp('OK\n\n');
    PosJ= mxNiSkeleton(KinectHandles);
else
    load(movementData)
end

%% Loading data for robot and connecting to robot
if(useRobot)
    loadRobotParam;
    [clientID,vrep] = connectToRobot(IPADDRESS, PORT, WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS);
    vrep.simxSynchronous(clientID,false);
    for jj = 1:nJoints
        [~, jointsHandle(jj)] = vrep.simxGetObjectHandle(clientID, cell2mat(jointsName(jj)), vrep.simx_opmode_oneshot_wait);
        tempLimit = jointsLimits(strcmp({jointsLimits.name},cell2mat(jointsName(jj))));
        upperLimits(jj) = tempLimit.upperLimit;
        lowerLimits(jj) = tempLimit.lowerLimit;
    end
    if(~vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot))
        disp('Error in starting simulation');
        return
    end
end

%% Initializing joint data for filtering; Y_(1) = Y_(2) = Y(1)
if(useKinect)
    while(PosJ(1)==0);
        mxNiUpdateContext(KinectHandles);
        %I=mxNiPhoto(KinectHandles); I=permute(I,[3 2 1]);
        PosJ= mxNiSkeleton(KinectHandles);
        %set(h,'Cdata',I); drawnow;
    end
    mxNiUpdateContext(KinectHandles);
    Pos1= mxNiSkeleton(KinectHandles);
    pause(0.01);
    mxNiUpdateContext(KinectHandles);
    Pos2= mxNiSkeleton(KinectHandles);
else
    Pos1 = squeeze(PosJ(1,:,:));
    Pos2 = squeeze(PosJ(2,:,:));
end

tempPos3D1 = Pos1(1:15,3:5)';
tempPos3D2 = Pos2(1:15,3:5)';
joints3D = [zeros(45,1),tempPos3D1(:),tempPos3D2(:)];
joints3D_ = [zeros(45,1),tempPos3D1(:),tempPos3D1(:)];
errors3D = [tempPos3D1(:)-tempPos3D1(:), tempPos3D2(:)-tempPos3D1(:)];
errors3D_ = errors3D;

%% Initializing plotting
hh=zeros(1,9);
figure
hold on
%set(gca,'YDir','reverse');
set(gca,'XDir','reverse');
%set(gca,'ZDir','reverse');
axis([-1000 1500 -1000 1500 1000 3200])

% %Da commentare
% xList(1) = Pos1(4,3);
% yList(1) = Pos1(4,4);
% zList(1) = Pos1(4,5);
% xList(2) = Pos2(4,3);
% yList(2) = Pos2(4,4);
% zList(2) = Pos2(4,5);
% joints3D__ = tempPos3D1(:)';
% xList_(1:2) = joints3D_(10,2:end);
% yList_(1:2) = joints3D_(11,2:end);
% zList_(1:2) = joints3D_(12,2:end);
% xList__(1) = joints3D__(10);
% yList__(1) = joints3D__(11);
% zList__(1) = joints3D__(12);
% xList__(2) = joints3D__(10);
% yList__(2) = joints3D__(11);
% zList__(2) = joints3D__(12);

zz = 1;
%% Main loop
%for kk = 3:size(PosJ,1)
for kk = 3:500
    
    %% Retieving data
    if(useKinect)
        mxNiUpdateContext(KinectHandles);
        %I=mxNiPhoto(KinectHandles); I=permute(I,[3 2 1]);
        %set(h,'Cdata',I); drawnow;
        Pos= mxNiSkeleton(KinectHandles,1);
    else
        Pos = squeeze(PosJ(kk,:,:));
    end
    tempPos3D = Pos(1:15,3:5)';
    joints3D = [joints3D(:,2:3), tempPos3D(:)];
    %% Filtering new data
    [tempVec,errors3D]= filterData(joints3D,errors3D,a1);
    joints3D_ = [joints3D_(:,2:3), tempVec];
    [joints3D__,errors3D_]= filterData(joints3D_,errors3D_,a2);
    
    %   %Da commentare
    %   xNonFilt= joints3D(1:3:end);
    %   yNonFilt= joints3D(2:3:end);
    %   zNonFilt= joints3D(3:3:end);
    %   xList(kk) = Pos(4,3);
    %   yList(kk) = Pos(4,4);
    %   zList(kk) = Pos(4,5);
    %   xList_(kk) = joints3D_(10,end);
    %   yList_(kk) = joints3D_(11,end);
    %   zList_(kk) = joints3D_(12,end);
    %   xList__(kk) = joints3D__(10);
    %   yList__(kk) = joints3D__(11);
    %   zList__(kk) = joints3D__(12);
    x3D_= joints3D__(1:3:end);
    y3D_= joints3D__(2:3:end);
    z3D_= joints3D__(3:3:end);
    
    %% Plotting skeleton in 3d graph
    if(hh(1)>0);
        for i=1:size(hh,2)
            delete(hh(i));
        end
    end
    hh(1)=plot3(x3D_,y3D_,z3D_,'rO');
    hh(2)=plot3(x3D_([13 14 15]),y3D_([13 14 15]),z3D_([13 14 15]),'g');%leftLeg
    hh(3)=plot3(x3D_([10 11 12]),y3D_([10 11 12]),z3D_([10 11 12]),'g');%rightLeg
    hh(4)=plot3(x3D_([9 10]),y3D_([9 10]),z3D_([9 10]),'m');%upperLeftLeg
    hh(5)=plot3(x3D_([9 13]),y3D_([9 13]),z3D_([9 13]),'m');%upperRightLeg
    hh(6)=plot3(x3D_([2 3 4 5]),y3D_([2 3 4 5]),z3D_([2 3 4 5]),'b');%leftArm
    hh(7)=plot3(x3D_([2 6 7 8]),y3D_([2 6 7 8]),z3D_([2 6 7 8]),'b');%rightArm
    hh(8)=plot3(x3D_([1 2]),y3D_([1 2]),z3D_([1 2]),'c');%neck
    hh(9)=plot3(x3D_([2 9]),y3D_([2 9]),z3D_([2 9]),'c--');%torso
    drawnow
    
    %% Computing links
    links(1,:) = [x3D_(1),y3D_(1), z3D_(1)] - [x3D_(2), y3D_(2), z3D_(2)];%head
    links(2,:) = [x3D_(9),y3D_(9), z3D_(9)] - [x3D_(2),y3D_(2), z3D_(2)];%head
    
    links(3,:) = [x3D_(2),y3D_(2), z3D_(2)] - [x3D_(3),y3D_(3), z3D_(3)];%leftShoulder
    links(4,:) = [x3D_(4),y3D_(4), z3D_(4)] - [x3D_(3),y3D_(3), z3D_(3)];%leftUpperArm
    links(5,:) = [x3D_(5),y3D_(5), z3D_(5)] - [x3D_(4),y3D_(4), z3D_(4)];%leftLowerArm
    
    links(6,:) = [x3D_(2),y3D_(2), z3D_(2)] - [x3D_(6),y3D_(6), z3D_(6)];%rightSholuderArm
    links(7,:) = [x3D_(6),y3D_(6), z3D_(6)] - [x3D_(7),y3D_(7), z3D_(7)];%rightUpperArm
    links(8,:) = [x3D_(7),y3D_(7), z3D_(7)] - [x3D_(8),y3D_(8), z3D_(8)];%rightLowerArm
    
    %% Computing joints angle
    [angles] = computeJointAngles(links); %shoulderPitch, shoulderRoll, eRoll, eYaw
    
    %% Sending data to robot
    if(useRobot)
        moveRobot(jointsHandle, angles(1:nJoints), upperLimits, lowerLimits, vrep, clientID, 0.01);
    end
    
    pause(0.01)
    zz = zz+1;
end

%% Closing connection to kinect
mxNiDeleteContext(KinectHandles);

%% Closing connection to robot
if(useRobot)
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
    vrep.simxFinish(clientID);
end