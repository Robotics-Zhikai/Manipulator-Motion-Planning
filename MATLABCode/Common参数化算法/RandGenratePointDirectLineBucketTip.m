function [PointA,BeginAngelBucketWithGround,PointB,EndAngelBucketWithGround,angleA,angleB] = RandGenratePointDirectLineBucketTip(Range) %不一定直线可达 在机械臂的平面内
    distance = 2*Range(2);
    GlobalDeclarationCommon
    while (distance<Range(1) || distance>Range(2))
        
        theta1 = RandGenerateNumber(theta1Range(1),theta1Range(2),1);

        theta2A = RandGenerateNumber(theta2Range(1),theta2Range(2),1);
        theta3A = RandGenerateNumber(theta3Range(1),theta3Range(2),1);
        theta4A = RandGenerateNumber(theta4Range(1),theta4Range(2),1);

        theta2B = RandGenerateNumber(theta2Range(1),theta2Range(2),1);
        theta3B = RandGenerateNumber(theta3Range(1),theta3Range(2),1);
        theta4B = RandGenerateNumber(theta4Range(1),theta4Range(2),1);

        angleA = [theta1 theta2A theta3A theta4A];
        angleB = [theta1 theta2B theta3B theta4B];
        
        [~,position1A] = ForwardKinematics(angleA);
        [~,position1B] = ForwardKinematics(angleB);

        PointA = position1A(1:3,4)';
        PointB = position1B(1:3,4)';
        
        distance = norm(PointA-PointB);
    end
    BeginAngelBucketWithGround = GetAngleOfBucketWithGround(theta1,theta2A,theta3A,theta4A);
    EndAngelBucketWithGround = GetAngleOfBucketWithGround(theta1,theta2B,theta3B,theta4B);
end
