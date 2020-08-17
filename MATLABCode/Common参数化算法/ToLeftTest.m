function result = ToLeftTest(PointA,PointB,PointC)
%这个函数当共线时，可能会判断错误，Area的值非常小，-10次方量级
    Area = PointA(1) * PointB(2) - PointA(2) * PointB(1) + PointB(1) * PointC(2) ...
		- PointB(2) * PointC(1) + PointC(1) * PointA(2) - PointC(2) * PointA(1);
    yuzhi = 1e-10;
    if Area > yuzhi
        result = 1;
    end
    if Area < -yuzhi
        result = -1;
    end
    if Area<yuzhi && Area>-yuzhi
        result = 0;
    end
end
