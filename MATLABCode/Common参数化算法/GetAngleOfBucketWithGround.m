function Angle = GetAngleOfBucketWithGround(theta1,theta2,theta3,theta4) %�õ����ĸ��ǵ�����²��������ļн� theta4���ڸ����ķ�Χ

%����AngleӦ����-180 180 ���Ӽ�
    GlobalDeclarationCommon
    k1 = theta1*pi/180;
    k2 = theta2*pi/180;
%     k3 = theta3*pi/180;
%     k4 = theta4*pi/180;
    
    [position1,position2] = ForwardKinematics([theta1,theta2,theta3,theta4]);
    
%     P3minusP2 = [230*cos(k1 + k2) + 230*cos(k1 - k2) ,230*sin(k1 - k2) + 230*sin(k1 + k2), 460*sin(k2)];%���Ǹ���ת�������������� T30-T20
    P3minusP2(1,1) = a2*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + d3*sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + d3*cos(m2)*sin(k1)*sin(m1);
    P3minusP2(1,2) = a2*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + d3*sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - d3*cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1));
    P3minusP2(1,3) = a2*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) - d3*sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) + d3*cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1));
    
    XPositive = [P3minusP2(1:2) 0]; %���������Ϊx�������� �����������180�ȸ���ʱ�������������
    XPositive = XPositive/norm(XPositive);
    
    BucketVector = position2(1:3,4)-position1(1:3,4);
    BucketVector = BucketVector/norm(BucketVector);
    %���� X ������ [-1, 1] �ڵ�ʵ��ֵ��acos(X) �������� [0, ��] �ڵ�ֵ��
    abstheta4 = acos(dot(BucketVector,XPositive));
    if BucketVector(3)<0
        Angle = -abstheta4;
    else
        Angle = abstheta4;
    end
    Angle = Angle*180/pi;
    Angle = legalizAnger(Angle);
end
