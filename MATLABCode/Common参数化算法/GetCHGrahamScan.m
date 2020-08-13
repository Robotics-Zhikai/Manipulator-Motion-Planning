function outPoints = GetCHGrahamScan(Points)
%     vector <Point> GetCHGrahamScan(vector<Point> Points)
%     {
        LTL = FindLowestThenLeftmostPoint(Points);
%         Point LTL = FindLowestThenLeftmostPoint(Points);
%         vector <Point> T_Stack;

        T_Stack = PreSorting(Points, LTL);% //主要耗费时间耗费在排序这里了

        NextEP = T_Stack(end,:);
%         Point NextEP = *(T_Stack.end()-1);
        T_Stack(end,:)=[];
%         T_Stack.pop_back();
%         vector <Point> S_Stack;
        S_Stack = [];
        S_Stack = [S_Stack;LTL];
        S_Stack = [S_Stack;NextEP];
%         S_Stack.push_back(LTL);
%         S_Stack.push_back(NextEP);
        while (isempty(T_Stack) == 0)
            A = S_Stack(end-1,:);
            B = S_Stack(end,:);
            C = T_Stack(end,:);
%             A = *(S_Stack.end() - 2);
%             B = *(S_Stack.end() - 1);
%             C = *(T_Stack.end() - 1);
            test = ToLeftTest(A, B, C);
            if (test == 0)
%                 if abs(A(1)-553.7642)<0.001
%                     disp('');
%                     ToLeftTest(A, B, C);
%                 end
                ifinli = IfInLine(A, B, C);
                if (ifinli == 1 || ifinli == -2 || ifinli == -3)
                    T_Stack(end,:)=[];
%                     T_Stack.pop_back();
                else
                    if (ifinli == -1)
                        S_Stack = [S_Stack;C];
                        T_Stack(end,:) = [];
%                         S_Stack.push_back(C);
%                         T_Stack.pop_back();
                    end
                end
            else
                if (test == 1)
                    S_Stack = [S_Stack;C];
                    T_Stack(end,:)=[];
%                     S_Stack.push_back(C);
%                     T_Stack.pop_back();
                else
                    if (test == -1)
                        S_Stack(end,:) = [];
%                         S_Stack.pop_back();
                    end
                end
            end
        end
%         vector<int> eraseIndex;% //删除共线的点内部的所有点
        eraseIndex = [];
        for i=1:size(S_Stack,1)-2
            if (ToLeftTest(S_Stack(i,:), S_Stack(i + 1,:), S_Stack(i + 2,:)) == 0)
                eraseIndex = [eraseIndex;i+1];
%                 eraseIndex.push_back(i + 1);
            end
        end

        S_Stack(eraseIndex,:)=[];
%         S_Stack = EraseVectorPoints(S_Stack, eraseIndex);
        outPoints = S_Stack;
%         return S_Stack;
%     }
end


function outPoint = FindLowestThenRightmostPoint(Points)
% Point FindLowestThenRightmostPoint(vector <Point> Points)
% //找到最低和最右的点
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

    xmax = yminPoints(1,1);
    result = yminPoints(1,:);
    for i=2:size(yminPoints,1)
        if yminPoints(i,1)>xmax
            result = yminPoints(i,:);
			xmax = yminPoints(i,1);
        end
    end
    outPoint = result;
end


function result = IfInLine(A,B,C)
% int IfInLine(Point A, Point B,Point C) 
% //判断C在共AB组成的直线的基础上是否在两端点内
% //如果在，那么return 1
% //否则 如果在AB方向上之外 return -1
% //如果在BA方向上之外，return -3
% //如果等于A或者B return -2
    if (norm(A - C)<0.001 || norm(B - C)<0.001)
        result =-2;
		return ;
    end
    if (A(1) == B(1))
        if ((C(2)<B(2) && C(2)>A(2)) || (C(2)<A(2) && C(2)>B(2)))
            result = 1;
            return;
% 			return 1;
        else
            if (C(2) > B(2))
                result = -1;
                return;
%                 return -1;
            else
                if (C(2) < A(2))
                    result = -3;
                    return;
%                     return -3;
                end
            end
        end
    else
        if ((C(1)<B(1) && C(1)>A(1)) || (C(1)<A(1) && C(1)>B(1)))
            result = 1;
            return;
% 			return 1;
        else
            if (C(1) > B(1))
                result = -1;
                return;
            else
                if (C(1) < A(1))
                    result = -3;
                    return;
                end
            end
        end
    end
end


