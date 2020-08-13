function YES = InConvexPolygonTest(Points,A)
% int InConvexPolygonTest(vector<Point> Points,Point A)
% //�жϵ�A�Ƿ���Points���ɵ�͹�������
% //Points������Ǽ��㼯����Ҫ���ظ�
% //O(n)
% //���ڲ�return1 �ڱ��� return0 ���ⲿ return-1 Error return-2 
% {
    if (size(Points,1) == 1)
        if ( norm(A - Points(1,:))<0.001)
            YES = 0;
            return;
        else
            YES = -1;
            return;
        end
    end
    if (size(Points,1) == 2)
        if (norm(Points(1,:) - Points(2,:))<0.001)
            if (norm(A - Points(1,:))<0.001)
                YES = 0;
                return;
            else
                YES = -1;
                return;
            end
        else
			result = ToLeftTest(Points(1,:), Points(2,:),A);
            if (result == 1 || result == -1)
                YES = -1;
                return;
            else
                if (result == 0)
                    YES = 0;
                    return;
                else
                    YES = -2;
                    return;
                end
            end
        end
    end
    if (CheckSorted(Points)==0)
% 		Points = BubbleSortPoints(Points);
        LTL = FindLowestThenLeftmostPoint(Points);
        Points = PreSorting(Points, LTL);% //��Ҫ�ķ�ʱ��ķ�������������
    end
% 	Points.push_back(Points[0]);
    Points = [Points;Points(1,:)];
% 	vector<int> testresult;
    testresult = [];
    for i = 0:size(Points,1)-2
        if (norm(Points(i+1,:) - Points(i + 2,:))<0.001)
			continue;
        end
		currentresult = ToLeftTest(Points(i+1,:), Points(i+2,:), A);
        testresult = [testresult currentresult];
% 		testresult.push_back(currentresult);
    end
	last = testresult(1);

	count0 = 0;
    flagbreak = 0;
    for i = 0:size(testresult,2)-1
        if (testresult(i+1) == 0)
            count0 = count0+1;
        end
        if (testresult(i+1) ~= 0 && testresult(i+1) ~= last && last~=0)
            flagbreak = 1;
			break;
        end
        if (testresult(i+1)~=0)
			last = testresult(i+1);
        end
    end
    
    if flagbreak==0
        if (count0 >= 1)
            YES = 0;
            return;
        else
            YES = 1;
            return;
        end
    else
        YES = -1;
        return;
    end
end