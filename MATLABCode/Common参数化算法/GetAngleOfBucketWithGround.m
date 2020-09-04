function Angle = GetAngleOfBucketWithGround(theta1,theta2,theta3,theta4) %得到在四个角的情况下铲斗与地面的夹角 theta4属于给定的范围

%最终Angle应该是-180 180 的子集
    GlobalDeclarationCommon
    k1 = theta1*pi/180;
    k2 = theta2*pi/180;
%     k3 = theta3*pi/180;
%     k4 = theta4*pi/180;
    
    [position1,position2] = ForwardKinematics([theta1,theta2,theta3,theta4]);
    
%     P3minusP2 = [230*cos(k1 + k2) + 230*cos(k1 - k2) ,230*sin(k1 - k2) + 230*sin(k1 + k2), 460*sin(k2)];%这是根据转换矩阵计算出来的 T30-T20
    P3minusP2(1,1) = a2*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + d3*sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + d3*cos(m2)*sin(k1)*sin(m1);
    P3minusP2(1,2) = a2*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + d3*sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - d3*cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1));
    P3minusP2(1,3) = a2*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) - d3*sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) + d3*cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1));
    
    XPositive = [P3minusP2(1:2) 0]; %以这个向量为x的正方向 当铲斗与其呈180度附近时，土不会掉下来
    XPositive = XPositive/norm(XPositive);
    
    BucketVector = position2(1:3,4)-position1(1:3,4);
    BucketVector = BucketVector/norm(BucketVector);
    %对于 X 在区间 [-1, 1] 内的实数值，acos(X) 返回区间 [0, π] 内的值。
    abstheta4 = acos(dot(BucketVector,XPositive));
    if BucketVector(3)<0
        Angle = -abstheta4;
    else
        Angle = abstheta4;
    end
    Angle = Angle*180/pi;
    Angle = legalizAnger(Angle);
end
