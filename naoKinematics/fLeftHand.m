function [Tendend, Derivatives, pos, or] = fLeftHand(thetas)

global shoulderOffsetY
global shoulderOffsetZ
global elbowOffsetY
global LowerArmLength
global HandOffsetX
global upperArmLength
global HandOffsetZ

base = eye(4,4);
base(2,4) = shoulderOffsetY;
base(3,4) = shoulderOffsetZ;
Derivatives=zeros(4,4,4);

T1 = DH(0,-pi/2,0,thetas(1));
dT1 = DHDerivative(0,-pi/2,0,thetas(1));
T2 = DH(0,pi/2,0,thetas(2)+pi/2);
dT2 = DHDerivative(0,pi/2,0,thetas(2)+pi/2);
T3 = DH(elbowOffsetY,pi/2,upperArmLength,thetas(3));
dT3 = DHDerivative(elbowOffsetY,pi/2,upperArmLength,thetas(3));
T4 = DH(0,-pi/2,0,thetas(4));
dT4 = DHDerivative(0,-pi/2,0,thetas(4));

R = RotXYZMatrix(0,0,-pi/2);

Tend1 = eye(4,4);
Tend1(1,4) = HandOffsetX+LowerArmLength;
Tend1(3,4) = -HandOffsetZ;
Tend = R*Tend1;
Tendend = base*T1*T2*T3*T4*Tend;

pos = Tendend(1:3,4)';
or(1) = atan2(Tendend(3,2),Tendend(3,3));
or(2) = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2+Tendend(3,3)^2));
or(3) = atan2(Tendend(2,1),Tendend(1,1));

Derivatives(1,:,:)=base*dT1*T2*T3*T4*Tend*Tend1;
Derivatives(2,:,:)=base*T1*dT2*T3*T4*Tend*Tend1;
Derivatives(3,:,:)=base*T1*T2*dT3*T4*Tend*Tend1;
Derivatives(4,:,:)=base*T1*T2*T3*dT4*Tend*Tend1;
end
