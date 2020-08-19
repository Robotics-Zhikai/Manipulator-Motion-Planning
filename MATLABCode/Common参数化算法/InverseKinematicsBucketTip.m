function result = InverseKinematicsBucketTip(pos,ThetaBucketwithGround) 
%����Ϊ�����ݼ��λ�úͲ��������ļнǣ����Ϊtheta1 theta2 theta3 theta4
%20200804��Ҫд
    ThetaBucketwithGround = legalizAnger(ThetaBucketwithGround);
    pos = pos';
    GlobalDeclarationCommon
%     qiedian = GetQiedianOfCircle([0,0],[pos(1),pos(2)],d2);
    gedian = GetGedianOfcircle([pos(1),pos(2)]);
    if isempty(gedian)==0
%         qiedian = qiedian(1,:);
    else
        erorr('�߼�����');
    end
    
    flagsuccess = 0;
    countNotReliable = 0;
    countNotFitable = 0;
    for i =1:size(gedian,1)
        direction = (pos(1,1:2)-gedian(i,1:2))/norm(pos(1,1:2)-gedian(i,1:2));
        if ThetaBucketwithGround<0
            thetatmp = ThetaBucketwithGround+90;
            pos3(1,1:2) = pos(1,1:2) - tool*sin(thetatmp*pi/180)*direction;
            pos3(1,3) = pos(1,3) + tool*cos(thetatmp*pi/180);
        else
            thetatmp = 90-ThetaBucketwithGround;
            pos3(1,1:2) = pos(1,1:2) - tool*sin(thetatmp*pi/180)*direction;
            pos3(1,3) = pos(1,3) - tool*cos(thetatmp*pi/180);
        end
        [theta123,reliable] = InverseKinematicsPos2Angle(pos3);
        if reliable==0
            countNotReliable = countNotReliable + 1;
            continue;
        end

        [Theta4,YES] = groundAngleRangeTOtheta4Range(theta123(1),theta123(2),theta123(3),[ThetaBucketwithGround,ThetaBucketwithGround]);
        if YES == 0
            countNotFitable = countNotFitable +1;
            continue;
%             error('��ǰ����ĳݼ�pos��������ʹ�ò��������н���ThetaBucketwithGround');
        else
            [postmp0,postmp] = ForwardKinematics([theta123(1),theta123(2),theta123(3),Theta4{1}]);
            if norm(postmp(1:3,4)-pos')>0.001
                continue;
            else
                flagsuccess = 1;
                break;
            end
        end
    end
    if countNotReliable==2 || countNotFitable==2
        result = [];
        return ; %�����޷����
    end
%     figure
%     PlotTheta1234(theta123(1),theta123(2),theta123(3),Theta4{1});
%     hold on 
%     plot3(pos(1),pos(2),pos(3),'o');
    if flagsuccess==0
        if CheckInrange(theta123(1),theta1Range) && CheckInrange(theta123(2),theta2Range) && CheckInrange(theta123(3),theta3Range)
            error('�����߼�����');
        else
            error('�����߼�û���⣬ָ����posλ�������Ƶ������²��ܵ���');
        end
    end
    result = [theta123(1),theta123(2),theta123(3),Theta4{1}];
end

function gedian = GetGedianOfcircle(ptOutside)
%Բ��������0,0��
    GlobalDeclarationCommon
    athis = norm(ptOutside);
    bthis = sqrt(a1^2+d2^2);
    xp = ptOutside(1);
    yp = ptOutside(2);
    cthis = (2*bthis*cosinetheta+sqrt(4*bthis^2*cosinetheta^2-4*(bthis^2-athis^2)))/2;
    cosalpha1 = (2*(bthis-cthis*cosinetheta)*xp + sqrt( (2*(bthis-cthis*cosinetheta)*xp)^2-4*athis^2*((bthis-cthis*cosinetheta)^2-yp^2) ) )/(2*athis^2);
    sinalpha1 = (bthis-cthis*cosinetheta-xp*cosalpha1)/yp;
    gedian(1,1)=bthis*cosalpha1;
    gedian(1,2)=bthis*sinalpha1;
    
    cosalpha2 = (2*(bthis-cthis*cosinetheta)*xp - sqrt( (2*(bthis-cthis*cosinetheta)*xp)^2-4*athis^2*((bthis-cthis*cosinetheta)^2-yp^2) ) )/(2*athis^2);
    sinalpha2 = (bthis-cthis*cosinetheta-xp*cosalpha2)/yp;
    gedian(2,1)=bthis*cosalpha2;
    gedian(2,2)=bthis*sinalpha2;
end

