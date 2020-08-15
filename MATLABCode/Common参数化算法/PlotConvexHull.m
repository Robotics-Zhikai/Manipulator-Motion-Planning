function PlotConvexHull(Points,PointType,LineType)
    if isempty(Points)==1
        return;
    end
    Points = GetCHGrahamScan(Points);
    plot(Points(:,1),Points(:,2),LineType);
    hold on 
    plot([Points(end,1),Points(1,1)],[Points(end,2),Points(1,2)],LineType);
    hold on 
    for i=1:size(Points,1)
        plot(Points(i,1),Points(i,2),PointType);
        hold on
    end
end