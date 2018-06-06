function quaternion = IMU_Newton( Ax , Ay, Az, Mx, My, Mz,epsilon, max_iteration ) 
   

syms a b c d ;
    ax_inertial = 0;
	ay_inertial = 0;
	az_inertial = 9.81;
	mx_inertial = 25.97;
	my_inertial = 9.25;
	mz_inertial = -9.14;
    iteration=0 ; 
 


 

   Gradient_Q = [ (2*(2*My*a - 2*Mx*b + 2*Mz*d)*(my_inertial - Mx*(2*a*b + 2*c*d) + Mz*(2*a*d - 2*b*c) + My*(a^2 - b^2 + c^2 - d^2)) - 2*(2*Mx*c - 2*Mz*a + 2*My*d)*(mz_inertial - Mx*(2*a*c - 2*b*d) - My*(2*a*d + 2*b*c) + Mz*(a^2 + b^2 - c^2 - d^2)) + 2*(2*Ax*a + 2*Ay*b + 2*Az*c)*(Ay*(2*a*b - 2*c*d) - ax_inertial + Az*(2*a*c + 2*b*d) + Ax*(a^2 - b^2 - c^2 + d^2)) + 2*(2*Mx*a + 2*My*b + 2*Mz*c)*(My*(2*a*b - 2*c*d) - mx_inertial + Mz*(2*a*c + 2*b*d) + Mx*(a^2 - b^2 - c^2 + d^2)) + 2*(2*Ay*a - 2*Ax*b + 2*Az*d)*(ay_inertial - Ax*(2*a*b + 2*c*d) + Az*(2*a*d - 2*b*c) + Ay*(a^2 - b^2 + c^2 - d^2)) - 2*(2*Ax*c - 2*Az*a + 2*Ay*d)*(az_inertial - Ax*(2*a*c - 2*b*d) - Ay*(2*a*d + 2*b*c) + Az*(a^2 + b^2 - c^2 - d^2))) ;
                  (2*(2*Mz*b - 2*My*c + 2*Mx*d)*(mz_inertial - Mx*(2*a*c - 2*b*d) - My*(2*a*d + 2*b*c) + Mz*(a^2 + b^2 - c^2 - d^2)) - 2*(2*Mx*a + 2*My*b + 2*Mz*c)*(my_inertial - Mx*(2*a*b + 2*c*d) + Mz*(2*a*d - 2*b*c) + My*(a^2 - b^2 + c^2 - d^2)) + 2*(2*Ay*a - 2*Ax*b + 2*Az*d)*(Ay*(2*a*b - 2*c*d) - ax_inertial + Az*(2*a*c + 2*b*d) + Ax*(a^2 - b^2 - c^2 + d^2)) + 2*(2*My*a - 2*Mx*b + 2*Mz*d)*(My*(2*a*b - 2*c*d) - mx_inertial + Mz*(2*a*c + 2*b*d) + Mx*(a^2 - b^2 - c^2 + d^2)) - 2*(2*Ax*a + 2*Ay*b + 2*Az*c)*(ay_inertial - Ax*(2*a*b + 2*c*d) + Az*(2*a*d - 2*b*c) + Ay*(a^2 - b^2 + c^2 - d^2)) + 2*(2*Az*b - 2*Ay*c + 2*Ax*d)*(az_inertial - Ax*(2*a*c - 2*b*d) - Ay*(2*a*d + 2*b*c) + Az*(a^2 + b^2 - c^2 - d^2))) ;
                  (-2*(2*Mx*a + 2*My*b + 2*Mz*c)*(mz_inertial - Mx*(2*a*c - 2*b*d) - My*(2*a*d + 2*b*c) + Mz*(a^2 + b^2 - c^2 - d^2)) - 2*(2*Mz*b - 2*My*c + 2*Mx*d)*(my_inertial - Mx*(2*a*b + 2*c*d) + Mz*(2*a*d - 2*b*c) + My*(a^2 - b^2 + c^2 - d^2)) - 2*(2*Ax*c - 2*Az*a + 2*Ay*d)*(Ay*(2*a*b - 2*c*d) - ax_inertial + Az*(2*a*c + 2*b*d) + Ax*(a^2 - b^2 - c^2 + d^2)) - 2*(2*Mx*c - 2*Mz*a + 2*My*d)*(My*(2*a*b - 2*c*d) - mx_inertial + Mz*(2*a*c + 2*b*d) + Mx*(a^2 - b^2 - c^2 + d^2)) - 2*(2*Ax*a + 2*Ay*b + 2*Az*c)*(az_inertial - Ax*(2*a*c - 2*b*d) - Ay*(2*a*d + 2*b*c) + Az*(a^2 + b^2 - c^2 - d^2)) - 2*(2*Az*b - 2*Ay*c + 2*Ax*d)*(ay_inertial - Ax*(2*a*b + 2*c*d) + Az*(2*a*d - 2*b*c) + Ay*(a^2 - b^2 + c^2 - d^2))) ;
                  (2*(2*Az*b - 2*Ay*c + 2*Ax*d)*(Ay*(2*a*b - 2*c*d) - ax_inertial + Az*(2*a*c + 2*b*d) + Ax*(a^2 - b^2 - c^2 + d^2)) - 2*(2*Mx*c - 2*Mz*a + 2*My*d)*(my_inertial - Mx*(2*a*b + 2*c*d) + Mz*(2*a*d - 2*b*c) + My*(a^2 - b^2 + c^2 - d^2)) - 2*(2*My*a - 2*Mx*b + 2*Mz*d)*(mz_inertial - Mx*(2*a*c - 2*b*d) - My*(2*a*d + 2*b*c) + Mz*(a^2 + b^2 - c^2 - d^2)) + 2*(2*Mz*b - 2*My*c + 2*Mx*d)*(My*(2*a*b - 2*c*d) - mx_inertial + Mz*(2*a*c + 2*b*d) + Mx*(a^2 - b^2 - c^2 + d^2)) - 2*(2*Ay*a - 2*Ax*b + 2*Az*d)*(az_inertial - Ax*(2*a*c - 2*b*d) - Ay*(2*a*d + 2*b*c) + Az*(a^2 + b^2 - c^2 - d^2)) - 2*(2*Ax*c - 2*Az*a + 2*Ay*d)*(ay_inertial - Ax*(2*a*b + 2*c*d) + Az*(2*a*d - 2*b*c) + Ay*(a^2 - b^2 + c^2 - d^2))) ];


Hessian_Q = [ diff(Gradient_Q(1),a)  diff(Gradient_Q(1),b)  diff(Gradient_Q(1),c)  diff(Gradient_Q(1),d);
              diff(Gradient_Q(2),a)  diff(Gradient_Q(2),b)  diff(Gradient_Q(2),c)  diff(Gradient_Q(2),d);
              diff(Gradient_Q(3),a)  diff(Gradient_Q(3),b)  diff(Gradient_Q(3),c)  diff(Gradient_Q(3),d);
              diff(Gradient_Q(4),a)  diff(Gradient_Q(4),b)  diff(Gradient_Q(4),c)  diff(Gradient_Q(4),d)];
          
quaternion = [ 0 0 0 1 ] ; 
iteration=0 ; 
E = 1000;
while ((iteration<=max_iteration)||(E < epsilon))
        Gradient_Q =  double(subs(Gradient_Q,[a b c d],[quaternion(1)  quaternion(2)  quaternion(3)  quaternion(4)])); 
	    Hessian_Q  =  double(subs(Hessian_Q ,[a b c d],[quaternion(1)  quaternion(2)  quaternion(3)  quaternion(4)]));
		quaternion = quaternion - (inv(Hessian_Q)*Gradient_Q)';
		E = Gradient_Q(1)*Gradient_Q(1) + Gradient_Q(2)*Gradient_Q(2) + Gradient_Q(3)*Gradient_Q(3) + Gradient_Q(4)*Gradient_Q(4);
		iteration = iteration + 1;
end
        



end

