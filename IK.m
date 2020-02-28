function Q = IK(P,m,robot)
   l0 = robot.links(1,1); l1 = robot.links(2,1); l2 = robot.links(3,1);
   l3 = robot.links(4,1); l4 = robot.links(5,1); l5 = robot.links(6,1); l6 = robot.links(7,1);
   
   if ~robot.orientation
       l4 = l4 + l5 + l6;
       l5 = 0; l6 = 0;
   end
      
   T = Tx(P(1))*Ty(P(2))*Tz(P(3))*Rx(P(4))*Ry(P(5))*Rz(P(6));
   
   % reduce tool and base
   T = Tz(l0) \ (T / Tx(l5+l6));        
   
   q1 = atan2(T(2,4), T(1,4));
           
   l34 = l3+l4;
   
   xy2 = m(1)*sqrt(T(1,4)^2+T(2,4)^2)-l1;
   
   [q2,q3] = solve2link(l2,l34,xy2,-T(3,4),m(2)); 
    
   if robot.orientation  
     T = (Rz(q1)*Tx(l1)*Ry(q2)*Tx(l2)*Ry(q3)*Tx(l34)) \ T;
     q5 = m(3)*acos(T(1,1));
     if ~ isreal(q5)
       q4 = 1i; q6 = 1i;
     elseif abs(T(1,1)) > 1e-6
      q4=atan2(m(3)*T(2,1),  -m(3)*T(3,1));
      q6=atan2(m(3)*T(1,2),  m(3)*T(1,3));
     else
      q4 = acos(T(2,2));
      q6 = 0;
     end
   else
       q4 = 0; q5 = 0; q6 = 0;
   end
   
   Q = check_limits([q1;q2;q3;q4;q5;q6], robot);   
end