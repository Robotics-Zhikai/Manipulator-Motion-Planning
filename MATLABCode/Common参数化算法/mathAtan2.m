
function result = mathAtan2(y,x)
    GlobalDeclarationCommon
    ZERO = ZeroDefine;
    if (abs(y) < ZERO)
		y = 0.0;
    end
    if (abs(x) < ZERO)
        x = 0.0;
    end
    result = atan2(y, x);
end