function YES = InTriangle(PointA, PointB, PointC,PointD) 
% int InTriangle(Point PointA, Point PointB, Point PointC,Point PointD) 
% //Is D in Triangle ABC.
% //if in,return 1.if not a triangle,return -2.if on triangle,return 0.if out of triangle,return -1.
	partA = ToLeftTest(PointA, PointB, PointD);
	partB = ToLeftTest(PointB, PointC, PointD);
	partC = ToLeftTest(PointC, PointA, PointD);
    if (partA == -2 || partB == -2 || partC == -2)
        YES = -2;
		return ;
    end
    if (partA == 1 && partB == 1 && partC == 1)
        YES = 1;
		return ;
    else
        if ((partA == 0 && partB==1 && partC==1 )|| (partB == 0 && partA == 1 && partC == 1) || ...
			(partC == 0 && partA == 1 && partB == 1)|| (partA == 0 && partB == 0 && partC == 1) || ...
			(partA == 0 && partC == 0 && partB == 1)|| (partB == 0 && partC == 0 && partA == 1))
            YES = 0;
			return ;
        else
            YES = -1;
			return ;	
        end
    end
end