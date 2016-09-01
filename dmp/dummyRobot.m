function [yNew] = dummyRobot(yDes, yActual,dt,t)


%error = error + dt*(yDes-yActual+error);
v = 100*(yDes-yActual);
yNew = yActual+ v*dt;
  if (t> 100 && t<250)
           yNew = yActual;

 end