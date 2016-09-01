function [position, positionReal] = myTest(IPADDRESS, PORT, WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS)

[clientID,vrep] = connectToRobot(IPADDRESS, PORT, WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS);

if (clientID>-1)
    disp('Connected to remote API server');
	vrep.simxSynchronous(clientID,false);
   
    [~, RShoulderPitch] = vrep.simxGetObjectHandle(clientID, 'RShoulderPitch3', vrep.simx_opmode_oneshot_wait);
    [~, RShoulderRoll] = vrep.simxGetObjectHandle(clientID, 'RShoulderRoll3', vrep.simx_opmode_oneshot_wait);
    [~, RElbowYaw] = vrep.simxGetObjectHandle(clientID, 'RElbowYaw3', vrep.simx_opmode_oneshot_wait);
    [~, RElbowRoll] = vrep.simxGetObjectHandle(clientID, 'RElbowRoll3', vrep.simx_opmode_oneshot_wait);
    
    if(~vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot))
        disp('Error in starting simulation');
        return
    end
    
    [res, RShoulderPitchAngle] = vrep.simxGetJointPosition(clientID,RShoulderPitch,vrep.simx_opmode_streaming);
    [res1, RShoulderRollAngle] = vrep.simxGetJointPosition(clientID,RShoulderRoll,vrep.simx_opmode_streaming);
    [res2, RElbowYawAngle] = vrep.simxGetJointPosition(clientID,RElbowYaw,vrep.simx_opmode_streaming);
    [res3, RElbowRollAngle] = vrep.simxGetJointPosition(clientID,RElbowRoll,vrep.simx_opmode_streaming);
    
    while(res & res1 & res2 & res3)
        [res, RShoulderPitchAngle] = vrep.simxGetJointPosition(clientID,RShoulderPitch,vrep.simx_opmode_buffer);
        [res1, RShoulderRollAngle] = vrep.simxGetJointPosition(clientID,RShoulderRoll,vrep.simx_opmode_buffer);
        [res2, RElbowYawAngle] = vrep.simxGetJointPosition(clientID,RElbowYaw,vrep.simx_opmode_buffer);
        [res3, RElbowRollAngle] = vrep.simxGetJointPosition(clientID,RElbowRoll,vrep.simx_opmode_buffer);
    end
    thetas = [0;0;0;degtorad(2)];
    
    vrep.simxSetJointTargetPosition(clientID, RShoulderPitch, thetas(1), vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID, RShoulderRoll, thetas(2), vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID, RElbowYaw, thetas(3), vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetPosition(clientID, RElbowRoll, thetas(4), vrep.simx_opmode_oneshot);
    z = 0;

    pause(0.5)
    
    while z < 1000
        [Tendend, ~, ~, ~] = fRightHand(thetas);
        TendendNew = Tendend; 
        
        TendendNew(1,4) = TendendNew(1,4) - 0.2*sin(0.7*pi*(z/1000));
        TendendNew(2,4) = TendendNew(2,4) + z/3000;
        TendendNew(3,4) = TendendNew(3,4) + 0.2*sin(0.5*pi*(z/1000));
        
        [thetas]=JacobianInverse(TendendNew,@fRightHand,thetas);
        vrep.simxSetJointTargetPosition(clientID, RShoulderPitch, thetas(1), vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetPosition(clientID, RShoulderRoll, thetas(2), vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetPosition(clientID, RElbowYaw, thetas(3), vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetPosition(clientID, RElbowRoll, thetas(4), vrep.simx_opmode_oneshot);
        z=z+1;
        pause(0.001)
        RShoulderPitchAngleReal(z) = thetas(1);
        RShoulderRollAngleReal(z) = thetas(2);
        RElbowYawAngleReal(z) = thetas(3);
        RElbowRollAngleReal(z) = thetas(4);
        [~, RShoulderPitchAngle(z)] = vrep.simxGetJointPosition(clientID,RShoulderPitch,vrep.simx_opmode_buffer);
        [~, RShoulderRollAngle(z)] = vrep.simxGetJointPosition(clientID,RShoulderRoll,vrep.simx_opmode_buffer);
        [~, RElbowYawAngle(z)] = vrep.simxGetJointPosition(clientID,RElbowYaw,vrep.simx_opmode_buffer);
        [~, RElbowRollAngle(z)] = vrep.simxGetJointPosition(clientID,RElbowRoll,vrep.simx_opmode_buffer);
        %pause
        %[Tendend, ~, ~, ~] = fRightHand(thetas);
    end
    % stop the simulation:
	vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
	% Now close the connection to V-REP:	
    vrep.simxFinish(clientID);
else
	disp('Failed connecting to remote API server');
end
vrep.delete();
positionReal = [RShoulderPitchAngleReal; RShoulderRollAngleReal;RElbowYawAngleReal;RElbowRollAngleReal];
position = [RShoulderPitchAngle;RShoulderRollAngle;RElbowYawAngle;RElbowRollAngle];
disp('Program ended');
end
