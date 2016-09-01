function [clientID,vrep] = connectToRobot(IPADDRESS, PORT, WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS)

vrep=remApi('remoteApi');
vrep.simxFinish(-1); % just in case, close all opened connections
clientID = vrep.simxStart(IPADDRESS, PORT, WAIT_UNTIL_CONNECTED, DO_NOT_RECONNECT_ONCE_DISCONNECTED, TIME_OUT_IN_MSEC, COMM_THREAD_CYCLE_IN_MS);

end