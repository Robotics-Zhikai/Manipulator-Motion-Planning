function outPoint = FindLowestThenLeftmostPoint(Points)
%     Point FindLowestThenLeftmostPoint(vector <Point> Points)
%     //找到最低和最左的点
        if (size(Points,1) == 0)
            outPoint = [0,0,0];
            return;
        end
        if size(Points,1)==1
            outPoint = Points(1,:);
            return;
        end
        ymin = Points(1,2);
        YminIndex = 0;
        yminPoints = [];
        yminPoints = [yminPoints;Points(1,:)];

        for i=2:size(Points,1)
            if Points(i,2)<ymin
                ymin = Points(i,2);
                YminIndex = i;
                yminPoints=[];
                yminPoints = [yminPoints;Points(i,:)];
            else
                if Points(i,2)==ymin
                    yminPoints = [yminPoints;Points(i,:)];
                end
            end
        end
        
        xmin = yminPoints(1,1);
        result = yminPoints(1,:);
        for i=2:size(yminPoints,1)
            if yminPoints(i,1)<xmin
                result = yminPoints(i,:);
                xmin = yminPoints(i,1);
            end
        end
        outPoint = result;
end
