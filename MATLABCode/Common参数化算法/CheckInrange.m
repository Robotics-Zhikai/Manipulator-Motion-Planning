function YES = CheckInrange(qujian,range) %���������qujian(2)>=qujian(1) range(2)>=range(1)
    if size(range,2)<=1
        error('range����Ϊ��������ߵ�����');
        YES = 0;
        return;
    end
    if isempty(qujian)==1
        error('qujian ����Ϊ������');
        YES = 0;
        return;
    end
    if size(qujian,2)==1
        if qujian>=range(1) && qujian <=range(2)
            YES = 1;
        else
            YES = 0;
        end
    else
        if qujian(2)<qujian(1) || range(2)<range(1)
            YES = 0;
        else
            if qujian(1)>=range(1)-0.001 && qujian(2)<=range(2)+0.001 
                YES = 1;
            else
                YES = 0;
            end
        end
    end
end
