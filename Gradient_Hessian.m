syms a b c d 
syms Ax Ay Az Mx My Mz ax_inertial ay_inertial az_inertial mx_inertial my_inertial mz_inertial
   M= [ (d^2+a^2-b^2-c^2)  2*(a*b-c*d)        2*(a*c+b*d)         0                  0                  0 ; 
        2*(a*b+c*d)        (d^2-a^2+b^2-c^2)  2*(b*c-a*d)         0                  0                  0 ;
        2*(a*c-b*d)        2*(b*c+a*d)        (d^2+c^2-b^2-a^2)   0                  0                  0 ;
         0                  0                  0                 (d^2+a^2-b^2-c^2)  2*(a*b-c*d)        2*(a*c+b*d) ;
         0                  0                  0                 2*(a*b+c*d)        (d^2-a^2+b^2-c^2)  2*(b*c-a*d) ;
         0                  0                  0                 2*(a*c-b*d)        2*(b*c+a*d)        (d^2+c^2-b^2-a^2) ];
 
 y0= [ Ax;Ay;Az;Mx;My;Mz];
 y1= [ ax_inertial;ay_inertial;az_inertial;mx_inertial;my_inertial;mz_inertial];
 
     
%%%Use formular%%%
% Gradient_Q = [-2*(da*y0).'*(y1-M*y0)
%               -2*(db*y0).'*(y1-M*y0)
%               -2*(dc*y0).'*(y1-M*y0)
%               -2*(dd*y0).'*(y1-M*y0)]
%           
% Temp1=[(da*y0).'
%        (db*y0).'
%        (dc*y0).'
%        (dd*y0).'];
% Temp2=[(da*y0)  (db*y0)  (dc*y0) (dd*y0)];
% Hessian_Q = -2*[ (d2aa*y0).'*(y1-M*y0)  (d2ab*y0).'*(y1-M*y0)  (d2ac*y0).'*(y1-M*y0)  (d2ad*y0).'*(y1-M*y0); 
%                  (d2ba*y0).'*(y1-M*y0)  (d2bb*y0).'*(y1-M*y0)  (d2bc*y0).'*(y1-M*y0)  (d2bd*y0).'*(y1-M*y0); 
%                  (d2ca*y0).'*(y1-M*y0)  (d2cb*y0).'*(y1-M*y0)  (d2cc*y0).'*(y1-M*y0)  (d2cd*y0).'*(y1-M*y0); 
%                  (d2da*y0).'*(y1-M*y0)  (d2db*y0).'*(y1-M*y0)  (d2dc*y0).'*(y1-M*y0)  (d2dd*y0).'*(y1-M*y0)]...
%                  + 2*(Temp1)*(Temp2)
 
%%%Use definition%%%
Q = (y1-M*y0).'*(y1-M*y0);
Gradient_Q =   [diff(Q,a) ;
                diff(Q,b) ;
                diff(Q,c) ;
                diff(Q,d)]

Hessian_Q = [ diff(Gradient_Q(1),a)  diff(Gradient_Q(1),b)  diff(Gradient_Q(1),c)  diff(Gradient_Q(1),d) ;
              diff(Gradient_Q(2),a)  diff(Gradient_Q(2),b)  diff(Gradient_Q(2),c)  diff(Gradient_Q(2),d) ; 
              diff(Gradient_Q(3),a)  diff(Gradient_Q(3),b)  diff(Gradient_Q(3),c)  diff(Gradient_Q(3),d) ;
              diff(Gradient_Q(4),a)  diff(Gradient_Q(4),b)  diff(Gradient_Q(4),c)  diff(Gradient_Q(4),d) ]
          
        
  
          quaternion = [ 0 0 0 1 ] ; 
         
          
Gradient_Q =  subs(Gradient_Q,[a b c d],[quaternion(1)  quaternion(2)  quaternion(3)  quaternion(4)])    
 
 Hessian_Q  =  subs(Hessian_Q ,[a b c d],[quaternion(1)  quaternion(2)  quaternion(3)  quaternion(4)])
       
		quaternion = quaternion - ((inv(Hessian_Q))*Gradient_Q)'
        


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 








