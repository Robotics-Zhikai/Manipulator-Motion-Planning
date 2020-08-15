function YES = IsTwoSegmentsIntersection(SegmentA,SegmentB)
%�˺��������ʹ��
%YES Ϊ1ʱ�����н��㣬���������������Ҳ������1����0ʱ����û�н���
    tmp1 = ToLeftTest(SegmentA.L,SegmentA.R,SegmentB.L);
    tmp2 = ToLeftTest(SegmentA.L,SegmentA.R,SegmentB.R);
    tmp3 = ToLeftTest(SegmentB.L,SegmentB.R,SegmentA.L);
    tmp4 = ToLeftTest(SegmentB.L,SegmentB.R,SegmentA.R);
    
    if (tmp1==tmp2 && (tmp1==1||tmp1==-1)) || (tmp3==tmp4 && (tmp3==1||tmp4==-1))
        YES = 0;
    else
        if tmp1==0 && tmp2==0 && tmp3==0 && tmp4==0
            if abs(SegmentA.L(1)-SegmentA.R(1))<1e-8
                tmpintersection = GetIntersection([SegmentA.L(2) SegmentA.R(2)],[SegmentB.L(2) SegmentB.R(2)]);
            else
                tmpintersection = GetIntersection([SegmentA.L(1) SegmentA.R(1)],[SegmentB.L(1) SegmentB.R(1)]);
            end
            if isempty(tmpintersection)==1
                YES = 0;
            else
                YES = 1;
            end
        else
            YES = 1;
        end
        
    end
end