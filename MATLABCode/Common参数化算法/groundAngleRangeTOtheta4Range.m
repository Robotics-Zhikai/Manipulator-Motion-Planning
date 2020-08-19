function [Theta4Range,YES] = groundAngleRangeTOtheta4Range(theta1,theta2,theta3,WithGroundAngleRange) %WithGroundAngleRange����Ϊ���ޣ���Ϊ���ޣ���ʱ��Ϊ��������180������Ϊ�� 
    %YES Ϊ1 ʱ������theta4��������ķ�Χ�����������ҳ�һ��Χ��������WithGroundAngleRange
    %WithGroundAngleRange�ĽǶ���ϵҲ��-180 180
    GlobalDeclarationCommon
    WithGroundAngleRange(1) = legalizAnger(WithGroundAngleRange(1));
    WithGroundAngleRange(2) = legalizAnger(WithGroundAngleRange(2));
    Theta4Range = [];
    BucketwithGroundRange = GetBucketwithGroundRange(theta1,theta2,theta3);
    SetBucketwithGroundRange = [];
    if BucketwithGroundRange(2) > BucketwithGroundRange(1)
        SetBucketwithGroundRange{1,1} = BucketwithGroundRange;
    else
        if BucketwithGroundRange(2) < BucketwithGroundRange(1)
            theta4mid = theta4Range(1) + 180-BucketwithGroundRange(1); %�ⲻ��130�й�ϵ
            SetBucketwithGroundRange{1,1} = [BucketwithGroundRange(1) 180];
            SetBucketwithGroundRange{2,1} = [theta4Range(1) theta4mid];
            SetBucketwithGroundRange{1,2} = [-179.99999 BucketwithGroundRange(2)];
            SetBucketwithGroundRange{2,2} = [theta4mid theta4Range(2)];
        end
    end
    
    SetWithGroundAngleRange = [];
    if WithGroundAngleRange(2) >= WithGroundAngleRange(1)
        SetWithGroundAngleRange{1,1} = WithGroundAngleRange;
    else
        if WithGroundAngleRange(2) < WithGroundAngleRange(1)
            SetWithGroundAngleRange{1,1} = [WithGroundAngleRange(1) 180];
            SetWithGroundAngleRange{1,2} = [-179.99999 WithGroundAngleRange(2)];
        end
    end
    
    intersectionSet = [];
    for i =1:size(SetBucketwithGroundRange,2)
        for j = 1:size(SetWithGroundAngleRange,2)
            intersecttmp = GetIntersection(SetBucketwithGroundRange{1,i},SetWithGroundAngleRange{1,j});
            if isempty(intersecttmp) == 0
                intersectionSet = [intersectionSet {intersecttmp}];
            end
        end
    end
    
    if BucketwithGroundRange(2)>BucketwithGroundRange(1)
        for i = 1:size(intersectionSet,2)
            tmp = intersectionSet{1,i};
%             -100 BucketwithGroundRange(1)
%             30 BucketwithGroundRange(2)
            if size(tmp,2)==2
                Theta4Range =[Theta4Range {[theta4Range(2) - (BucketwithGroundRange(2)-tmp(1)), theta4Range(2) - (BucketwithGroundRange(2)-tmp(2))]}];
            else
                if size(tmp,2)==1
                    Theta4Range =[Theta4Range {theta4Range(2) - (BucketwithGroundRange(2)-tmp(1))}];
                else
                    erro('�߼�����');
                end
            end
        end 
    else
        if BucketwithGroundRange(2)<BucketwithGroundRange(1)
            for i=1:size(SetBucketwithGroundRange,2)
                for j = 1:size(intersectionSet,2)
                    if isempty(GetIntersection(intersectionSet{1,j},SetBucketwithGroundRange{1,i}))==0
                        if size(intersectionSet{1,j},2)==1
                            Theta4Range = [Theta4Range {SetBucketwithGroundRange{2,i}(1)+intersectionSet{1,j}(1)-SetBucketwithGroundRange{1,i}(1)}];
                        else
                            Theta4Range = [Theta4Range {[SetBucketwithGroundRange{2,i}(1)+intersectionSet{1,j}(1)-SetBucketwithGroundRange{1,i}(1) , ...
                            SetBucketwithGroundRange{2,i}(1)+intersectionSet{1,j}(2)-SetBucketwithGroundRange{1,i}(1)]}];
                        end
                    end
                end
            end
%             Mode = 2;
        end
    end
    
    if isempty(Theta4Range)==1
        YES = 0;
    else
        YES = 1;
    end
    if YES==1
        for i=1:size(Theta4Range,2)
            if CheckInrange(Theta4Range{i},theta4Range)==0
                error('�����߼�����');
            end
        end
    end

    
%     if isempty(Theta4Range)==1
%         YES = 0;
%     else
%         if size(Theta4Range,2)==1
%             Theta4Range = Theta4Range{1};
%             if CheckInrange(Theta4Range,theta4Range)==0
%                 error('�����߼�����');
%             end
%         else
%             if size(Theta4Range,2)==2
%                 if abs(Theta4Range{1}(2)-Theta4Range{2}(1))>0.001
%                     if CheckInrange(Theta4Range{1},theta4Range)==0
%                         error('�����߼�����');
%                     end
%                     if CheckInrange(Theta4Range{2},theta4Range)==0
%                         error('�����߼�����');
%                     end
%                 else
%                     Theta4Range = [Theta4Range{1}(1) Theta4Range{2}(2)];
%                     if CheckInrange(Theta4Range,theta4Range)==0
%                         error('�����߼�����');
%                     end
%                 end
%             else
% %                 error('�����߼�����');
%                 %�����п��ܳ������������
%                 for i=1:size(Theta4Range,2)
%                     if CheckInrange(Theta4Range{i},theta4Range)==0
%                         error('�����߼�����');
%                     end
%                 end
%             end
%         end
%         YES = 1;
%     end
%     if isempty(Theta4Range)==1 && YES == 1
%         error('�����߼�����');
%     end
end
