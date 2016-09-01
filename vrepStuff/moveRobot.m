function positions = moveRobot(jointsList, angularValues, upperLimits, lowerLimits, vrep, clientID,dt)
%% moveRobot: send the latest positions of specified joints to the robot.
%       positions = moveRobot(jointsList, angularValues, upperLimits, lowerLimits, vrep, clientID,dt)
%
vrep.simxPauseCommunication(clientID,1);
for ii = 1:length(jointsList)
    if(checkFeasibility(angularValues(ii), upperLimits(ii),lowerLimits(ii)))
        vrep.simxSetJointTargetPosition(clientID, jointsList(ii), angularValues(ii), vrep.simx_opmode_oneshot);
    else
        disp(['Movement of ',num2str(jointsList(ii)),' not feasible!']);
    end
end
vrep.simxPauseCommunication(clientID,0);
pause(dt*10)
for ii = 1:length(jointsList)
    [~, positions(ii)] = vrep.simxGetJointPosition(clientID, jointsList(ii), vrep.simx_opmode_buffer);
end

