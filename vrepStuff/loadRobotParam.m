IPADDRESS                           = '127.0.0.1';
PORT                                = 19997;
WAIT_UNTIL_CONNECTED                = true;
DO_NOT_RECONNECT_ONCE_DISCONNECTED  = true;
TIME_OUT_IN_MSEC                    = 3000;
COMM_THREAD_CYCLE_IN_MS             = 5;


%jointsName = [{'RShoulderPitch3'},{'RShoulderRoll3'},{'RElbowYaw3'},{'RElbowRoll3'}];
jointsName = [{'LShoulderPitch3'},{'LShoulderRoll3'},{'LElbowRoll3'}];

jointsLimits(1).name = 'LShoulderPitch3';
jointsLimits(1).upperLimit = degtorad(-1.195e+02+2.390e+02);
jointsLimits(1).lowerLimit = degtorad(-1.195e+02);
jointsLimits(2).name = 'LShoulderRoll3';
jointsLimits(2).upperLimit = degtorad(-1.800e+01+ 9.401e+01);
jointsLimits(2).lowerLimit = degtorad(-1.800e+01);
jointsLimits(3).name = 'LElbowYaw3';
jointsLimits(3).upperLimit = degtorad(-1.195e+02+2.390e+02);
jointsLimits(3).lowerLimit = degtorad(-1.195e+02);
jointsLimits(4).name = 'LElbowRoll3';
jointsLimits(4).upperLimit = degtorad(-8.850e+01+8.650e+01);
jointsLimits(4).lowerLimit = degtorad(-8.850e+01);
