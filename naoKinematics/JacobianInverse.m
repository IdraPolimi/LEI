function [thetas]=JacobianInverse(Target,forward,initial)

alpha=0.2;
%CLAMP=10;
MAXITER=500;
Mask=zeros(4,4);
Mask(1:3,4)=1;


[T, Derivatives, ~, ~]=forward(initial);
[k, l, m]=size(Derivatives);
%k chain size
%l,m = 4
J=ones(l*m,k);
TOL=10^-5;
initial=initial(1:k);

quality=sum(sum((abs(Target-T).*Mask)));
ITER=0;

while((quality>TOL && ITER<=MAXITER)|| ITER==0)
%while(quality>TOL)
	ITER=ITER+1;
	tplot(ITER,:)=initial;
	[T, Derivatives, ~, ~]=forward(initial);

	quality=sum(sum((abs(Target-T).*Mask)));

	

	%alpha=0.2
	%Target
	%T1
	e=(Target-T).*Mask;
	e=e(:);
	%e=e(1:end-4);
	%m=max(abs(e));
% 	if(m>CLAMP)
%         disp('CLAMP');
%         pause
% 		e=(e./m).*CLAMP;
% 	end
	
	for i=1:k		
        p=squeeze(Derivatives(i,:,:));%apla p = Derivatives(i,:,:) san return
        p=p(:);
        J(:,i)=p;
    end
    %s=sign(J'*e);
	%g=s*alpha*(abs(J'*e))^0.9;
    %size(J)
    %size(pinv(J))
    %size(e)
	g=alpha*pinv(J)*e;
	%g=alpha*J*e;
% 	s=abs(g)<1e-7;
% 	if(s)
%         
% 		g(s)=sign(g(s)).*(abs(g(s))).^0.6;
% 	end
	%g=(2./(1+exp(-g))-1)
	
    initial=initial+g;%.*[0 1]'		
	%pause
end
thetas=initial;
end