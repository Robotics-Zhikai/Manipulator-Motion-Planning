function YES = IsAnglesInLimitRange(jointAngle)
%����Ϊ�Ƕ�
    GlobalDeclarationCommon
    if isempty(GetIntersection(jointAngle(1),theta1Range)) || isempty(GetIntersection(jointAngle(2),theta2Range)) ||  isempty(GetIntersection(jointAngle(3),theta3Range)) || isempty(GetIntersection(jointAngle(4),theta4Range))
%         error('��ƵĹ滮�㷨ʹ�ýǶȳ�������������')
        YES = 0;
    else
        YES = 1;
    end
end