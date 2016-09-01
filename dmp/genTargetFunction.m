%each row of ytg,dytg and ddytg is a gld!

T = timeScale/(tau*dt);
% for jj = 1:gdl
%     for ii=1:T
%         ytg(jj,ii) = (exp(ii/1000)+sin(jj*pi*ii/1000)+0.5)/10;
%     end
%     dytgTemp =diff(ytg(jj,:))/dt;
%     ddytgTemp =diff(dytgTemp)/dt;
%     dytg(jj,:) = [dytgTemp(1) dytgTemp];
%     ddytg(jj,:) = [ddytgTemp(1) ddytgTemp(1) ddytgTemp];
% end
%%%%%%%%%%%%%%%%%
%zOld = 0;
for jj = 1:gdl
    %    k = 1;
    %    for ii=-(T/2):(T/2-1)
    %       z = ii/1000;
    %       ytg(jj,k) = (z^5 + 3.5*z^4 - 2-5*z^3 - 12.5*z^2 + 1.5*z +9)/10+jj;
    %       k = k+1;
    %       zOld = z;
    %   end
    ytg(jj,:) = positionsReal(jj,:);
    dytg(jj,:)=gradient(ytg(jj,:))/(1/1000);
    ddytg(jj,:)=gradient(dytg(jj,:))/(1/1000);
    %size(dytgTemp)
    %size(ddytgTemp)
    %dytg(jj,:) = [dytgTemp(1) dytgTemp];
    %ddytg(jj,:) = [ddytgTemp(1) ddytgTemp(1) ddytgTemp];
end
clear dytgTemp
clear ddytgTemp
clear jj
clear ii
