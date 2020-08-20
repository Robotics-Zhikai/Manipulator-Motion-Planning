function PeterCorkePlotRobot(jointSeqence)
%用Petercorke的方法绘制机器人 可视化
%输入是四个角度的列向量 角度值
    jointSeqence = deg2rad(jointSeqence);
    GlobalDeclarationCommon
    ML1 =  Link([ pi/4,      d1,   a0,       m0,        0], 'modified');
    ML2 =  Link([ deg2rad(theta2Range(1)),  d2, a1, m1,   0], 'modified');
    ML3 =  Link([ deg2rad(theta3Range(1)),  d3, a2, m2,       0], 'modified');
    ML4 =  Link([ deg2rad(theta4Range(1)),  d4, a3, m3,   0], 'modified');
    ML5 =  Link([ 0,      0,        tool,       0 ,    0], 'modified');

    ML1.qlim = [deg2rad(theta1Range(1)) deg2rad(theta1Range(2))];
    ML2.qlim = [deg2rad(theta2Range(1)) deg2rad(theta2Range(2))];
    ML3.qlim = [deg2rad(theta3Range(1)) deg2rad(theta3Range(2))];
    ML4.qlim = [deg2rad(theta4Range(1)) deg2rad(theta4Range(2))];
    ML5.qlim = [0 0];

    robotPeterCorke=SerialLink([ML1 ML2 ML3 ML4 ML5],'name','ROBOT');
    jointSeqence = [jointSeqence zeros(size(jointSeqence,1),1)];
    robotPeterCorke.plot(jointSeqence);
end