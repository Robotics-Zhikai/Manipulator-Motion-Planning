function CommonPoints = GetConvexHullIntersection(PointsA,PointsB)
%得到输入的两个点集分别组成的凸包的交集的点集
    if isempty(PointsA) || isempty(PointsB)
        CommonPoints = [];
        return;
    end
    PointsA = GetCHGrahamScan(PointsA);
    PointsB = GetCHGrahamScan(PointsB);
    
    
    CommonPoints = [];
    for i=1:size(PointsA,1)
        LAST = i;
        if i+1>size(PointsA,1)
            AFTER = 1;
        else
            AFTER = i+1;
        end
        seg1 = Segment(PointsA(LAST,:),PointsA(AFTER,:));
        for j = 1:size(PointsB,1)
            LASTB = j;
            if j+1>size(PointsB,1)
                AFTERB = 1;
            else
                AFTERB = j+1;
            end
            seg2 = Segment(PointsB(LASTB,:),PointsB(AFTERB,:));
            Points = GetTwoSegmentsIntersection(seg1,seg2);
            if size(Points,1)>1
                disp('有两条线段重叠了！');
                
            end
            CommonPoints = [CommonPoints;Points];
        end
    end
    
    for i=1:size(PointsB,1)
        YES = InConvexPolygonTest(PointsA,PointsB(i,:));
        if YES==1 || YES==0
            CommonPoints = [CommonPoints;PointsB(i,:)];
        end
    end
    
    for i=1:size(PointsA,1)
        YES = InConvexPolygonTest(PointsB,PointsA(i,:));
        if YES==1 || YES==0
            CommonPoints = [CommonPoints;PointsA(i,:)];
        end
    end
    if isempty(CommonPoints)==1
        CommonPoints = [];
    else
        CommonPoints = GetCHGrahamScan(CommonPoints);
    end
end

















