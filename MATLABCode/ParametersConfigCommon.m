% ParametersConfigCommon
global m0 m1 m2 m3 a0 a1 a2 a3 d1 d2 d3 d4 tool 
global theta1Range theta2Range theta3Range theta4Range ZeroDefine
global TwoInnerTangentPoints

m0=0*pi/180;
m1=90*pi/180;
m2=0*pi/180;
m3=0*pi/180;
a0=0;
d3=0;
d4=0;
a1=12;
a2=460;
a3=210.9;
d1=57.9;
d2=13.7;
tool=123.5;

theta1Range = [-179.9999 180];
theta2Range = [-40 44];
theta3Range = [-130 -20];
theta4Range = [-100 30];

ZeroDefine = 10^-6;

TwoInnerTangentPoints = FindInnerEdgeTangentPoints();

function TWOPOINTS = FindInnerEdgeTangentPoints()
    lidu = 0.5;
    OuterEdge = GetOuterEdgeOfPlaneWorkSpace(lidu);
    InnerEdge = GetInnerEdgeOfPlaneWorkSpace(lidu);
    
    figure
    plot(OuterEdge(:,1),OuterEdge(:,2),'.');
    hold on 
    plot(InnerEdge(:,1),InnerEdge(:,2),'.');
    hold on 
    
    InnerCH = [InnerEdge zeros(size(InnerEdge,1),1)];
    InnerCH = GetCHGrahamScan(InnerCH);
    plot(InnerCH(:,1),InnerCH(:,2),'-');
    hold on
    
    %找到相切的那个直线的两点
    TWOPOINTS = [];
    lastdistance = norm(InnerCH(1,:)-InnerCH(2,:));
    for i=2:size(InnerCH,1)-1
        currentdistance = norm(InnerCH(i,:)-InnerCH(i+1,:));
        if norm(currentdistance-lastdistance)>10*lastdistance && i~=size(InnerCH,1)-1
            TWOPOINTS = [TWOPOINTS;[InnerCH(i,:);InnerCH(i+1,:)]];
        end
        lastdistance = currentdistance;
    end
    if size(TWOPOINTS,1)~=2
        error('求内部凸包不能按照这种方法求！');
    end
    plot(TWOPOINTS(:,1),TWOPOINTS(:,2),'o');
    hold on 
    TWOPOINTS(:,3) = [];
    
    [A,B,C] = GetLineABC(TWOPOINTS(1,:),TWOPOINTS(2,:));
    y=[140,690];
    z=(-A*y-C)/B;
    plot(y,z,'-');
    hold on
end

function EdgePoints = GetOuterEdgeOfPlaneWorkSpace(lidu)
    GlobalDeclarationCommon
    EdgePoints = [];
    k3 = theta3Range(2);
%     lidu = 0.1;
    for k2=theta2Range(1):lidu:theta2Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end
    
    k2=theta2Range(1);
    for k3 = theta3Range(1):lidu:theta3Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end
end

function EdgePoints = GetInnerEdgeOfPlaneWorkSpace(lidu) %得到平面的内侧边缘 y-z系 
    GlobalDeclarationCommon
    EdgePoints = [];
    k3 = theta3Range(1);
%     lidu = 0.1;
    for k2=theta2Range(1):lidu:theta2Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end
    
    for k3 = theta3Range(1):lidu:theta3Range(2)
        [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
        EdgePoints = [EdgePoints;[position1(2,4),position1(3,4)]];
    end
    
%     for k2=theta2Range(1):5:theta2Range(2)
%         for k3 = theta3Range(1):5:theta3Range(2)
%             [position1,position2] = ForwardKinematics([90 k2 k3 theta4Range(1)]);
%             plot(position1(2,4),position1(3,4),'.');
%             hold on
%             pause(0.1);
%         end
%     end
end

function [A,B,C] = GetLineABC(P1,P2)
    X1 = P1(1); Y1 = P1(2); 
    X2 = P2(1); Y2 = P2(2); 
    A = Y2 - Y1;
    B = X1 - X2;
    C = X2*Y1 - X1*Y2;
end



% k1=pi/2;
% syms k2 k3
% T40_14 = a0 + a2*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + a1*cos(k1) + a3*(cos(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) - cos(m2)*sin(k3)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + sin(k1)*sin(k3)*sin(m1)*sin(m2)) + d4*cos(m3)*(sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + cos(m2)*sin(k1)*sin(m1)) + d2*sin(k1)*sin(m1) + d4*sin(m3)*(sin(k3)*(cos(k1)*cos(k2) - cos(m1)*sin(k1)*sin(k2)) + cos(k3)*cos(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) - cos(k3)*sin(k1)*sin(m1)*sin(m2)) + d3*sin(m2)*(cos(k1)*sin(k2) + cos(k2)*cos(m1)*sin(k1)) + d3*cos(m2)*sin(k1)*sin(m1);
% T40_24 = a2*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) - d1*sin(m0) - a3*(sin(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - cos(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(m2)*sin(k3)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + d4*sin(m3)*(sin(k3)*(cos(k2)*cos(m0)*sin(k1) - sin(k2)*sin(m0)*sin(m1) + cos(k1)*cos(m0)*cos(m1)*sin(k2)) + cos(k3)*sin(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) + cos(k3)*cos(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1))) + a1*cos(m0)*sin(k1) - d2*cos(m1)*sin(m0) + d3*sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) + d4*cos(m3)*(sin(m2)*(cos(m0)*sin(k1)*sin(k2) + cos(k2)*sin(m0)*sin(m1) - cos(k1)*cos(k2)*cos(m0)*cos(m1)) - cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1))) - d3*cos(m2)*(cos(m1)*sin(m0) + cos(k1)*cos(m0)*sin(m1)) - d2*cos(k1)*cos(m0)*sin(m1);
% T40_34 = d1*cos(m0) + a3*(cos(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + sin(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) + cos(m2)*sin(k3)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + a2*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + d2*cos(m0)*cos(m1) - d4*sin(m3)*(cos(k3)*sin(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - sin(k3)*(cos(k2)*sin(k1)*sin(m0) + cos(m0)*sin(k2)*sin(m1) + cos(k1)*cos(m1)*sin(k2)*sin(m0)) + cos(k3)*cos(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0))) + a1*sin(k1)*sin(m0) - d3*sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - d4*cos(m3)*(sin(m2)*(cos(k2)*cos(m0)*sin(m1) - sin(k1)*sin(k2)*sin(m0) + cos(k1)*cos(k2)*cos(m1)*sin(m0)) - cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1))) + d3*cos(m2)*(cos(m0)*cos(m1) - cos(k1)*sin(m0)*sin(m1)) - d2*cos(k1)*sin(m0)*sin(m1);
% H_T40_14 = hessian(T40_14,[k2,k3]);
% H_T40_24 = hessian(T40_24,[k2,k3]);
% H_T40_34 = hessian(T40_34,[k2,k3]);
% 
% H_T40_14 = simplify(H_T40_14);
% H_T40_24 = simplify(H_T40_24);
% H_T40_34 = simplify(H_T40_34);
% 
% EIG_T40_14 = eig(H_T40_14);
% EIG_T40_24 = eig(H_T40_24);
% EIG_T40_34 = eig(H_T40_34);
% 
% EIG_T40_14 = simplify(EIG_T40_14)
% EIG_T40_24 = simplify(EIG_T40_24)
% EIG_T40_34 = simplify(EIG_T40_34)
% i=0;
% stor = [];
% figure
% for k2 = theta2Range(1):30:theta2Range(2)
%     for k3 = theta3Range(1):0.1:theta3Range(2)
%         i=i+1;
%         k2 = k2*pi/180;
%         k3 = k3*pi/180;
% %         x1 = - (10477000778445366699*2^(1/2)*cos(k2 + k3 + pi/4))/811296384146066816957890051440640 - (4967757600021511*(9737881 - 4447881*sin(2*k2 + 2*k3) - 5290000*sin(2*k2))^(1/2))/811296384146066816957890051440640 - (571292124002473765*2^(1/2)*cos(k2 + pi/4))/40564819207303340847894502572032;
% %         x2 = (4967757600021511*(9737881 - 4447881*sin(2*k2 + 2*k3) - 5290000*sin(2*k2))^(1/2))/811296384146066816957890051440640 - (10477000778445366699*2^(1/2)*cos(k2 + k3 + pi/4))/811296384146066816957890051440640 - (571292124002473765*2^(1/2)*cos(k2 + pi/4))/40564819207303340847894502572032;
%         y1 = (2109*sin(k2)*sin(k3))/10 - (78771455770557675*sin(k2))/91343852333181432387730302044767688728495783936 - (52047200242553257367489300683062189*cos(k2)*sin(k3))/65820182292848241686198767302294020199309434625343194533944360960 - (288920000382793281*cos(k3)*sin(k2))/365375409332725729550921208179070754913983135744 - 230*cos(k2) - (136789681932384823947222935080863816294581548315651775905747201001861080051151819278132079035353661440*sin(2*k2) - 722491984224435937830171260347983221859323949217145915067991446112598108142133338729257518525575069696*sin(2*k3) + 36483091095989476386458040224006086019596702398100113726031079592418437733198717953353604677022517178830133171047914417286798643298304*cos(k2)^2 - 192695388308684126850570257184070676342571299613432225065403873239537064527908343863588920351223899344892560857509429031617980428471575*cos(k3)^2 + 385390776617368253701140514368141352685142599226864450130807746479074129055816687944509056964267413066696903900444891390591027273813271*cos(k2)^2*cos(k3)^2 + 2889967936897743693356411355885310543742895473192181241650105637678888301604743249255701396299303616512*cos(k2)*cos(k3)^2*sin(k2) + 2889967936897743693356411355885310543742895473192181241650105637678888301604743249255701396299303616512*cos(k2)^2*cos(k3)*sin(k3) - 385390776617368253701140514368141352685142599226864450130807746479074129055816687944509056964267417425723154318626440549835870381801472*cos(k2)*cos(k3)*sin(k2)*sin(k3) + 192695388308684126850570257184070676342571299613432225065403873245467765793198460586259524486305606894718680392079804090076134213943296)^(1/2)/65820182292848241686198767302294020199309434625343194533944360960 - (2109*cos(k2)*cos(k3))/10;
%         y2 = (2109*sin(k2)*sin(k3))/10 - (78771455770557675*sin(k2))/91343852333181432387730302044767688728495783936 - (52047200242553257367489300683062189*cos(k2)*sin(k3))/65820182292848241686198767302294020199309434625343194533944360960 - (288920000382793281*cos(k3)*sin(k2))/365375409332725729550921208179070754913983135744 - 230*cos(k2) + (136789681932384823947222935080863816294581548315651775905747201001861080051151819278132079035353661440*sin(2*k2) - 722491984224435937830171260347983221859323949217145915067991446112598108142133338729257518525575069696*sin(2*k3) + 36483091095989476386458040224006086019596702398100113726031079592418437733198717953353604677022517178830133171047914417286798643298304*cos(k2)^2 - 192695388308684126850570257184070676342571299613432225065403873239537064527908343863588920351223899344892560857509429031617980428471575*cos(k3)^2 + 385390776617368253701140514368141352685142599226864450130807746479074129055816687944509056964267413066696903900444891390591027273813271*cos(k2)^2*cos(k3)^2 + 2889967936897743693356411355885310543742895473192181241650105637678888301604743249255701396299303616512*cos(k2)*cos(k3)^2*sin(k2) + 2889967936897743693356411355885310543742895473192181241650105637678888301604743249255701396299303616512*cos(k2)^2*cos(k3)*sin(k3) - 385390776617368253701140514368141352685142599226864450130807746479074129055816687944509056964267417425723154318626440549835870381801472*cos(k2)*cos(k3)*sin(k2)*sin(k3) + 192695388308684126850570257184070676342571299613432225065403873245467765793198460586259524486305606894718680392079804090076134213943296)^(1/2)/65820182292848241686198767302294020199309434625343194533944360960 - (2109*cos(k2)*cos(k3))/10;
%         z1 = - (2109*sin(k2 + k3))/10 - 230*sin(k2) - (2^(1/2)*(9737881 - 4447881*cos(2*k2 + 2*k3) - 5290000*cos(2*k2))^(1/2))/20;
%         z2 = (2^(1/2)*(9737881 - 4447881*cos(2*k2 + 2*k3) - 5290000*cos(2*k2))^(1/2))/20 - 230*sin(k2) - (2109*sin(k2 + k3))/10;
%         
%         if mod(i,1)==0
%             subplot(161)
%             plot(i,y1,'.');
%             hold on 
%             subplot(162)
%             plot(i,y2,'.');
%             hold on 
%             subplot(163)
%             plot(i,z1,'.');
%             hold on 
%             subplot(164)
%             plot(i,z2,'.');
%             hold on 
%             
%             [position1,position2] = ForwardKinematics([k1*180/pi,k2*180/pi,k3*180/pi,0]);
%             subplot(165)
%             plot(i,position1(2,4),'.');
%             hold on 
%             subplot(166)
%             plot(i,position1(3,4),'.');
%             hold on 
%             pause(0.1)
%             hold on 
%         end
%         if  ((y1<0 && y2<0) || (y1>0 && y2>0)) || (( z1>0 && z2>0)||(z1<0 && z2<0)) %x1<0 || x2<0 ||
%             
%             stor = [stor;i];
%         else
%             
%         end
%     end
% end
% disp('');
% 
% function [position1,position2] = ForwardKinematics(jointangle)
% % 	double m[4], a[4], theta[4], d[4], tool;
% % 	double m_matrix1[16], m_matrix2[16], m_matrix3[16], m_matrix4[16], m_matrix5[16];
% % 	double m_matrixtemp1[16], m_matrixtemp2[16], m_matrixtemp3[16], m_matrixtemp4[16];
% % 	double m_dRPY[3];
% 
%     GlobalDeclarationCommon
% %     ZERO = 10^-6;
%     ZERO = ZeroDefine;
%     M_PI = pi;
%     M_PI_2 = pi/2;
% % 	m(1) = 0.0*M_PI / 180.0;
% % 	m(2) = 90.0*M_PI / 180.0;
% % 	m(3) = 0.0*M_PI / 180.0;
% % 	m(4) = 0.0*M_PI / 180.0;
%     m(1) = m0;
% 	m(2) = m1;
% 	m(3) = m2;
% 	m(4) = m3;
% 
% % 	a(1) = 0.0;
% % 	a(2) = 12.0;
% % 	a(3) = 460.0;
% % 	a(4) = 210.9;
%     a(1) = a0;
% 	a(2) = a1;
% 	a(3) = a2;
% 	a(4) = a3;
% 
% % 	d(1) = 57.9;
% % 	d(2) = 13.7;%//13.7
% % 	d(3) = 0.0;
% % 	d(4) = 0.0;
% % 	tool = 123.5;
%     d(1) = d1;
% 	d(2) = d2;
% 	d(3) = d3;
% 	d(4) = d4;
% 	tool = tool;
% 
% 	theta(1) = jointangle(1) * M_PI / 180.0;
% 	theta(2) = jointangle(2) * M_PI / 180.0;
% 	theta(3) = jointangle(3) * M_PI / 180.0;
% 	theta(4) = jointangle(4) * M_PI / 180.0;
% 
% 	m_matrix1 = MatrixT(m(1), a(1), theta(1), d(1));
% 	m_matrix2 = MatrixT(m(2), a(2), theta(2), d(2));
% 	m_matrix3 = MatrixT(m(3), a(3), theta(3), d(3));
% 	m_matrix4 = MatrixT(m(4), a(4), theta(4), d(4));
% 	m_matrix5 = MatrixTool(tool);
% 
%     m_matrixtemp1 = m_matrix1*m_matrix2;
%     m_matrixtemp2 = m_matrixtemp1*m_matrix3;
%     m_matrixtemp3 = m_matrixtemp2*m_matrix4;
%     m_matrixtemp4 = m_matrixtemp3*m_matrix5;
% 	
% 
% % 	//欧拉角A、B、C begin
% 	m_dRPY(2) = mathAtan2(-m_matrixtemp4(3,1), sqrt(m_matrixtemp4(1,1) * m_matrixtemp4(1,1) + m_matrixtemp4(2,1) * m_matrixtemp4(2,1)));%//欧拉角B
% 
%     if ((m_dRPY(2)<M_PI_2 + ZERO) && (m_dRPY(2)>M_PI_2 - ZERO))
%         m_dRPY(3) = mathAtan2(m_matrixtemp4(1,2), m_matrixtemp4(2,2));
% 		m_dRPY(1) = 0.0;
%     else
%         if ((m_dRPY(2)<-M_PI_2 + ZERO) && (m_dRPY(2)>-M_PI_2 - ZERO))
%             m_dRPY(3) = -mathAtan2(m_matrixtemp4(1,2), m_matrixtemp4(2,2));%//欧拉角A
%             m_dRPY(1) = 0.0;%//欧拉角C
%         else
%             m_dRPY(3) = mathAtan2(m_matrixtemp4(2,1) / cos(m_dRPY(2)), m_matrixtemp4(1,1) / cos(m_dRPY(2)));%//欧拉角A
%             m_dRPY(1) = mathAtan2(m_matrixtemp4(3,2) / cos(m_dRPY(2)), m_matrixtemp4(3,3) / cos(m_dRPY(2)));%//欧拉角C
%         end
%     end
% 
% % 	//欧拉角A、B、C end
% 
% % 	//printf("\nEuler%f %f %f\n", m_dRPY[0] * 180.0 / M_PI, m_dRPY[1] * 180.0 / M_PI, m_dRPY[2] * 180.0 / M_PI);
% 
%     position1 = m_matrixtemp3;%//铲斗旋转中心坐标
%     position2 = m_matrixtemp4;%//末端铲齿坐标
% end
% 
% 
% function TransMatrix = MatrixT(alpha, a, theta, d )
% 	TransMatrix(1,1) = cos(theta);
% 	TransMatrix(1,2) = -sin(theta);
% 	TransMatrix(1,3) = 0.0;
% 	TransMatrix(1,4) = a;
% 
% 	TransMatrix(2,1) = sin(theta)*cos(alpha);
% 	TransMatrix(2,2) = cos(theta)*cos(alpha);
% 	TransMatrix(2,3) = -sin(alpha);
% 	TransMatrix(2,4) = -sin(alpha)*d;
% 
% 	TransMatrix(3,1) = sin(theta)*sin(alpha);
% 	TransMatrix(3,2) = cos(theta)*sin(alpha);
% 	TransMatrix(3,3) = cos(alpha);
% 	TransMatrix(3,4) = cos(alpha)*d;
% 
% 	TransMatrix(4,1) = 0.0;
% 	TransMatrix(4,2) = 0.0;
% 	TransMatrix(4,3) = 0.0;
% 	TransMatrix(4,4) = 1.0;
% end
% 
% function matrixtool = MatrixTool(tool)
% 	matrixtool(1,1) = 1.0;
% 	matrixtool(1,2) = 0.0;
% 	matrixtool(1,3) = 0.0;
% 	matrixtool(1,4) = tool;
% 
% 	matrixtool(2,1) = 0.0;
% 	matrixtool(2,2) = 1.0;
% 	matrixtool(2,3) = 0.0;
% 	matrixtool(2,4) = 0.0;
% 
% 	matrixtool(3,1) = 0.0;
% 	matrixtool(3,2) = 0.0;
% 	matrixtool(3,3) = 1.0;
% 	matrixtool(3,4) = 0.0;
% 
% 	matrixtool(4,1) = 0.0;
% 	matrixtool(4,2) = 0.0;
% 	matrixtool(4,3) = 0.0;
% 	matrixtool(4,4) = 1.0;
% end
% 
% function result = mathAtan2(y,x)
%     GlobalDeclarationCommon
%     ZERO = ZeroDefine;
%     if (abs(y) < ZERO)
% 		y = 0.0;
%     end
%     if (abs(x) < ZERO)
%         x = 0.0;
%     end
%     result = atan2(y, x);
% end

