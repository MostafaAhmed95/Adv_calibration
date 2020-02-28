function X = FK(q,robot)
   T = Tz(robot.links(1,1))*Rz(q(1))*Tx(robot.links(2,1))*Ry(q(2))*Tx(robot.links(3,1))*Ry(q(3))*...
       Tx(robot.links(4,1))*Rx(q(4))*Tx(robot.links(5,1))*Ry(q(5))*Tx(robot.links(6,1))*Rx(q(6))*Tx(robot.links(7,1));
   b = asin(T(1,3));
   if abs(b-pi/2) < 1E-3
       c = 0;
       a = acos(T(2,2));
   else
       c = atan2(-T(1,2),T(1,1));
       a = atan2(-T(2,3),T(3,3));
   end
%    b = acos(T(1,1));
%    if abs(b)>1e-5
%       a=atan2(T(2,1),  -T(3,1));
%       c=atan2(T(1,2),  T(1,3));
%    else
%       a = acos(T(2,2));
%       c = 0;
%    end
   X = [T(1:3,4);a;b;c];
end