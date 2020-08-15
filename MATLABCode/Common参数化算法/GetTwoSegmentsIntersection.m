function Points = GetTwoSegmentsIntersection(SegmentA,SegmentB)
%获取两个线段的交点
    Points = [];
    if norm(SegmentA.L-SegmentA.R)==0 || norm(SegmentB.L-SegmentB.R)==0 
        error('某一线段长度为0');
        return;
    else
        if IsTwoSegmentsIntersection(SegmentA,SegmentB)==0
            Points = [];
        else
            if ToLeftTest(SegmentA.L,SegmentA.R,SegmentB.L)==0 && ToLeftTest(SegmentA.L,SegmentA.R,SegmentB.R)==0
                %说明有无穷多交点
                if abs(SegmentA.L(1)-SegmentA.R(1))<1e-8
                    tmp = [SegmentA.L(2) SegmentA.R(2) SegmentB.L(2) SegmentB.R(2)];
                else
                    tmp = [SegmentA.L(1) SegmentA.R(1) SegmentB.L(1) SegmentB.R(1)];
                end
                tmp1 = [{SegmentA.L} {SegmentA.R} {SegmentB.L} {SegmentB.R}];
                [~,minloc] = min(tmp);
                [~,maxloc] = max(tmp);
                standard = 1:size(tmp,2);
                sortedloc = sort([minloc maxloc]);
                IterationStandard = 1;
                Iterationsortedloc = 1;

                UniqueRecord = [];
                while Iterationsortedloc<size(sortedloc,2)+1 && IterationStandard<size(standard,2)+1
                    if standard(IterationStandard)~=sortedloc(Iterationsortedloc)
                        UniqueRecord = [UniqueRecord IterationStandard];
                        IterationStandard = IterationStandard+1;
                    else
                        Iterationsortedloc = Iterationsortedloc+1;
                        IterationStandard = IterationStandard+1;
                    end
                end
                
                for i=IterationStandard:size(standard,2)
                    UniqueRecord = [UniqueRecord i];
                end

                for i = 1:size(UniqueRecord,2)
                    Points = [Points;tmp1{UniqueRecord(i)}];
                end
            else
                [A1,B1,C1] = GetLineABC(SegmentA.L,SegmentA.R);
                [A2,B2,C2] = GetLineABC(SegmentB.L,SegmentB.R);
                INVAB = inv([A1 B1;A2 B2]);
                Points(1) = INVAB(1,1)*(-C1)+INVAB(1,2)*(-C2);
                Points(2) = INVAB(2,1)*(-C1)+INVAB(2,2)*(-C2);
                Points(3) = 0;
            end
        end
    end
end