function [Tendend, Derivatives, pos, or] = fRightLeg(thetas)
global HipOffsetZ
global HipOffsetY
global ThighLength
global TibiaLength
global FootHeight

base = eye(4,4);
base(2,4) = -HipOffsetY;
base(3,4) = -HipOffsetZ;
Derivatives=zeros(6,4,4);

T1 = DH(0,-pi/4,0,thetas(1)-pi/2);
dT1= DHDerivative(0,-pi/4,0,thetas(1)-pi/2);
T2 = DH(0,-pi/2,0,thetas(2)-pi/4);
dT2 = DHDerivative(0,-pi/2,0,thetas(2)-pi/4);
T3 = DH(0,pi/2,0,thetas(3));
dT3 = DHDerivative(0,pi/2,0,thetas(3));
T4 = DH(-ThighLength,0,0,thetas(4));
dT4 = DHDerivative(-ThighLength,0,0,thetas(4));
T5 = DH(-TibiaLength,0,0,thetas(5));
dT5 = DHDerivative(-TibiaLength,0,0,thetas(5));
T6 = DH(0,-pi/2,0,thetas(6));
dT6 = DHDerivative(0,-pi/2,0,thetas(6));

R = RotZYXMatrix(pi,-pi/2,0);%*RotXYZMatrix(pi/2,0,0);
Tend1 = eye(4,4);
Tend1(3,4) = -FootHeight;
Tend = R;
Tendend = base*T1*T2*T3*T4*T5*T6*Tend*Tend1;



pos = Tendend(1:3,4)';
or(1) = atan2(Tendend(3,2),Tendend(3,3));
or(2) = atan2(-Tendend(3,1),sqrt(Tendend(3,2)^2+Tendend(3,3)^2));
or(3) = atan2(Tendend(2,1),Tendend(1,1));

Derivatives(1,:,:)=base*dT1*T2*T3*T4*T5*T6*Tend*Tend1;
Derivatives(2,:,:)=base*T1*dT2*T3*T4*T5*T6*Tend*Tend1;
Derivatives(3,:,:)=base*T1*T2*dT3*T4*T5*T6*Tend*Tend1;
Derivatives(4,:,:)=base*T1*T2*T3*dT4*T5*T6*Tend*Tend1;
Derivatives(5,:,:)=base*T1*T2*T3*T4*dT5*T6*Tend*Tend1;
Derivatives(6,:,:)=base*T1*T2*T3*T4*T5*dT6*Tend*Tend1;
end
