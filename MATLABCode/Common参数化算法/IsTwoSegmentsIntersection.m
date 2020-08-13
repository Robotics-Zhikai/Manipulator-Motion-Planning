function YES = IsTwoSegmentsIntersection(SegmentA,SegmentB)
%此函数作检测使用
%YES 为1时表明有交点，交点可能是无穷多个也可能是1个；0时表明没有交点
    tmp1 = ToLeftTest(SegmentA.L,SegmentA.R,SegmentB.L);
    tmp2 = ToLeftTest(SegmentA.L,SegmentA.R,SegmentB.R);
    tmp3 = ToLeftTest(SegmentB.L,SegmentB.R,SegmentA.L);
    tmp4 = ToLeftTest(SegmentB.L,SegmentB.R,SegmentA.R);
    
    if (tmp1==tmp2 && (tmp1==1||tmp1==-1)) || (tmp3==tmp4 && (tmp3==1||tmp4==-1))
        YES = 0;
    else
        YES = 1;
    end
end