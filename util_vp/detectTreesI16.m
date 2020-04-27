function x=detectTrees(RR) 
% laser range  RR is a [1x361] matrix, in meters!!!!.
% return x, a 3xn matrix, where
% x(1,i) distance to i-landmark center
% x(2,i) angle of i-landmark center (in radians)
% x(3,i) i-landmark diameter
% José. ACFR. 1999.



% --------------------------------
M11 = 75 ;
M10 =  1 ;
daa = 5*pi/306 ;
M2=1.5 ; 	
M2a = 10*pi/360 ;
M3=3 ;		% *************** 
M5=1 ; % 1 mt   % *************** 
daMin2 = 2*pi/360 ;
% --------------------------------
global AAr; AA = AAr ;
ii1=Find16X(RR<M11) ;
L1 = length(ii1) ; 
if L1<1, x=[] ; return ; end ;

R1 = RR(ii1) ; A1 = AA(ii1) ;

ii2 = find(  (abs(diff(R1))>M2)|( diff(A1)>M2a) ) ;

L2= length(ii2)+1 ;
%ii2 = [1, ii2+1, ii1(L1) ] ;
ii2u = int16([ ii2, L1 ]) ;
ii2  = int16([1, ii2+1 ]) ;

%ii2 , size(R1) ,

R2  = R1(ii2 ) ; A2  = A1(ii2 ) ; 
A2u = A1(ii2u) ; R2u = R1(ii2u) ;

x2  =  R2.*cos(A2 ) ; y2 =  R2.*sin(A2 ) ;
x2u = R2u.*cos(A2u) ;y2u = R2u.*sin(A2u) ;

flag=zeros(1,L2) ;

L3=0 ;M3c= M3*M3 ;
if L2>1, 
 L2m=L2-1 ;
 dx2 = x2(2:L2)-x2u(1:L2m) ;  dy2 = y2(2:L2)-y2u(1:L2m) ; 
 %dl2 = sqrt( dx2.*dx2+dy2.*dy2 ) ;	%dont use SQRT
 dl2 = ( dx2.*dx2+dy2.*dy2 ) ;
 ii3 = find(dl2<M3c) ; L3=length(ii3) ;
 if L3>0, flag(ii3)=1 ; flag(ii3+1)=1 ; end ;

if L2>2, 
 L2m=L2-2 ;
 dx2 = x2(3:L2)-x2u(1:L2m) ;  dy2 = y2(3:L2)-y2u(1:L2m) ; 
 dl2 = ( dx2.*dx2+dy2.*dy2 ) ;
 ii3 = find(dl2<M3c) ;L3b=length(ii3) ;
 if L3b>0, flag(ii3)=1 ; flag(ii3+2)=1 ; L3=L3+L3b ; end ;

if L2>3, % OJOOOOOOOOO
 L2m=L2-3 ;
 dx2 = x2(4:L2)-x2u(1:L2m) ;  dy2 = y2(4:L2)-y2u(1:L2m) ; 
 dl2 = ( dx2.*dx2+dy2.*dy2 ) ;
 ii3 = find(dl2<M3c) ;L3b=length(ii3) ;
 if L3b>0, flag(ii3)=1 ; flag(ii3+3)=1 ;L3=L3+L3b ; end ;

end ;end ;end ;

if L2>1, 
 ii3 = [1:L2-1] ;
 ii3 = find( (A2(ii3+1)-A2u(ii3))<daMin2 ) ;%objects close (in angle) from viewpoint.
 L3b=length(ii3) ;
 if L3b>0,
    ff = (R2(ii3+1)>R2u(ii3)) ;		% which object is in the back?
	 ii3=ii3+ff	;
	 flag(ii3)=1	;		% mark them for the deletion
	 L3=L3+L3b	;
 end ;
 iixx=ii3 ;
end ;

if L3>0,
	ii3=Find16X(flag==0) ;
   L3=length(ii3) ;
   ii4 = double(ii2(ii3)) ; ii4u = double(ii2u(ii3)) ;
	R4  = R2(ii3)  ; R4u  = R2u(ii3)  ;
	A4  = A2(ii3)  ; A4u  = A2u(ii3)  ;
	x4  = x2(ii3)  ; y4   = y2(ii3)   ;
	x4u = x2u(ii3) ; y4u  = y2u(ii3)  ;
else
   ii4 = double(ii2) ; ii4u = double(ii2u);
	R4  = R2  ; R4u  = R2u ;
	A4  = A2  ; A4u  = A2u  ;
	x4  = x2  ; y4   = y2   ;
	x4u = x2u ; y4u  = y2u  ;
end ;

%dA2=diff(A2) ; dD2 = dA2.*R2(1:L2-1);
dx2 = x4-x4u ;  dy2 = y4-y4u ; 
dl2 = ( dx2.*dx2+dy2.*dy2 ) ;
ii5 = Find16X(dl2<(M5*M5)) ; L5 = length(ii5) ; if L5<1, x=[] ; return ; end ;

R5 = R4(ii5) ; R5u = R4u(ii5) ;	A5 = A4(ii5) ; A5u = A4u(ii5) ; ii4=ii4(ii5) ;ii4u=ii4u(ii5) ;

ii5=Find16X( (R5>M10)&(A5>daa)&(A5u<(pi-daa)) ) ;
L5 = length(ii5) ; if L5<1, x=[] ; return ; end ;
R5 = R5(ii5) ; R5u = R5u(ii5) ;	A5 = A5(ii5) ; A5u = A5u(ii5) ; ii4=ii4(ii5) ;ii4u=ii4u(ii5) ;
dL5 = (A5u+pi/360-A5).*(R5+R5u)/2  ; %/2/2

compa = ( abs(R5-R5u)<(dL5/3) ) ;	

ii6 = Find16X(~compa) ; ii6=ii4(ii6) ;

ii5 = Find16X(compa) ; L5 = length(ii5) ; if L5<1, x=[] ; return ; end ;
R5 = R5(ii5) ; R5u = R5u(ii5) ;	A5 = A5(ii5) ; A5u = A5u(ii5) ; ii4=ii4(ii5) ;ii4u=ii4u(ii5) ;
dL5 =dL5 (ii5) ;

auxi = (ii4+ii4u)/2 ;
iia= floor(auxi) ;
iib= ceil(auxi) ;
Rs = (R1(iia)+R1(iib))/2 ;
x = [ Rs+dL5/2;(A5+A5u)*0.5;dL5;] ; % distance , angle, size ;

%x = [(R5+R5u)*0.5;(A5+A5u)*0.5;dL5;] ;

return ;

% ---------------------------------------
function ii = Find16X(x)
	ii = int16(find(x)) ;	
return ;
% ---------------------------------------


