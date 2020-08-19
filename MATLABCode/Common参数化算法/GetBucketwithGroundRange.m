function BucketwithGroundRange = GetBucketwithGroundRange(theta1,theta2,theta3) 
%得到theta4满足给定的范围的前提下与地面的夹角范围

%     k1 = theta1*pi/180;
%     k2 = theta2*pi/180;
%     k3 = theta3*pi/180;
%     BucketVector = [- (247*cos(k4)*(cos(k3)*( - cos(k1)*cos(k2)) + sin(k3)*(cos(k1)*sin(k2) )))/2 - (247*sin(k4)*(cos(k3)*(cos(k1)*sin(k2) ) - sin(k3)*( - cos(k1)*cos(k2))))/2;
%    (247*cos(k4)*(cos(k3)*(  cos(k2)*sin(k1)) - sin(k3)*(sin(k1)*sin(k2) )))/2 - (247*sin(k4)*(cos(k3)*(sin(k1)*sin(k2) ) + sin(k3)*(  cos(k2)*sin(k1))))/2;
%     (247*cos(k4)*(cos(k2)*sin(k3) + cos(k3)*sin(k2)))/2 - (247*sin(k4)*(sin(k2)*sin(k3) - cos(k2)*cos(k3)))/2];
%     BucketVector = BucketVector/norm(BucketVector);
%     
%     P3minusP2 = [230*cos(k1 + k2) + 230*cos(k1 - k2) ,230*sin(k1 - k2) + 230*sin(k1 + k2), 460*sin(k2)];%这是根据转换矩阵计算出来的 T30-T20
%     XPositive = [P3minusP2(1:2) 0]; %以这个向量为x的正方向 当铲斗与其呈180度附近时，土不会掉下来
%     XPositive = XPositive/norm(XPositive);
%     figure;
%     StableRange = [-145 -179.99];
    GlobalDeclarationCommon
    
    angletheta4withground = [GetAngleOfBucketWithGround(theta1,theta2,theta3,theta4Range(1)),GetAngleOfBucketWithGround(theta1,theta2,theta3,theta4Range(2))];
    BucketwithGroundRange = angletheta4withground;
%     if abs(angletheta4withground(1)-angletheta4withground(2))>180
%         BucketwithGroundRange = [angletheta4withground(1) 360+angletheta4withground(2)];
%         %当-100,30 中经过计算有经过180时，会造成角度突变，此时将其角度范围弄到0-360，以保证连续
%     end
end
