function t = theta_jac(q, theta, param)
   t = zeros(6,param.theta_no);
   
   T_rot = Ry(-q(3))*Ry(-q(2))*Rz(-q(1));
   
   L1 = param.links(1,1); L2 = param.links(2,1); L3 = param.links(3,1);
   L4 = param.links(4,1); L5 = param.links(5,1); L6 = param.links(6,1);
   L7 = param.links(7,1);
   drx = dRx(0); dry = dRy(0); drz = dRz(0);
   dtx = dTx(0); dty = dTy(0); dtz = dTz(0);
   Tool = param.tool;
   
   
%    T = Tz(robot.links(1,1))*Rz(q(1))*Tx(robot.links(2,1))*Ry(q(2))*Tx(robot.links(3,1))*Ry(q(3))*...
%        Tx(robot.links(4,1))*Rx(q(4))*Tx(robot.links(5,1))*Ry(q(5))*Tx(robot.links(6,1))*Rx(q(6))*Tx(robot.links(7,1));

   
   % 1
   tmp = Tz(L1) * Rz(q(1)) * drz * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3))* Tool * T_rot; 
   t(:,1) = Jcol(tmp);
   % 2
   tmp = Tz(L1) * Rz(q(1)) * Tz(L2) * Tz(q(2))* dtz * Tx(L3) * Tx(q(3))* Tool * T_rot; 
   t(:,2) = Jcol(tmp);
   % 3
   tmp = Tz(L1) * Rz(q(1)) * Tz(L2) * Tz(q(2)) * Tx(L3) * Tx(q(3))* dtx * Tool * T_rot; ; 
   t(:,3) = Jcol(tmp);
  
   
      
end