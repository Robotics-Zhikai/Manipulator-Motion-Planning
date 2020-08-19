function YES = CheckInrange(qujian,range) %必须得满足qujian(2)>=qujian(1) range(2)>=range(1)
    if size(range,2)<=1
        error('range不能为空区间或者单个数');
        YES = 0;
        return;
    end
    if isempty(qujian)==1
        error('qujian 不能为空区间');
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
