function YES = IsAnglesInLimitRange(jointAngle)
%输入为角度
    GlobalDeclarationCommon
    if isempty(GetIntersection(jointAngle(1),theta1Range)) || isempty(GetIntersection(jointAngle(2),theta2Range)) ||  isempty(GetIntersection(jointAngle(3),theta3Range)) || isempty(GetIntersection(jointAngle(4),theta4Range))
%         error('设计的规划算法使得角度超出了物理限制')
        YES = 0;
    else
        YES = 1;
    end
end