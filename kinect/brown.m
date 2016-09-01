% clear all
% clc
% x = -2*pi:0.01:2*pi;
% Y = sin(x)+0.05*rand(1, length(x));
% 
% a = 0.1;
% Y_(1) = Y(1);
% Y_(2) = Y(1);
% e(1) = Y(1)-Y_(1);
% e(2) = Y(2)-Y_(2);
% for ii = 3:length(Y)
%     Y_(ii) = 2*Y(ii-1)-Y(ii-2)-2*(1-a)*e(ii-1)+(1-a)^2*e(ii-2);
%     if ii <= length(Y)
%         e(ii) = Y(ii)-Y_(ii);
%     end
% end
% plot(Y,'r-');
% hold on
% plot(Y_,'-');

function [Y_, e] = brown(Y, prevY, pprevY, prevE, pprevE, a)
   Y_ = 2*prevY-pprevY-2*(1-a)*prevE+(1-a)^2*pprevE;
   e = Y-Y_;
end