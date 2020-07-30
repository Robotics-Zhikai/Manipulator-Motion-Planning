function outPoints = GetCHGrahamScan(Points)
%     vector <Point> GetCHGrahamScan(vector<Point> Points)
%     {
        LTL = FindLowestThenLeftmostPoint(Points);
%         Point LTL = FindLowestThenLeftmostPoint(Points);
%         vector <Point> T_Stack;

        T_Stack = PreSorting(Points, LTL);% //��Ҫ�ķ�ʱ��ķ�������������

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
%         vector<int> eraseIndex;% //ɾ�����ߵĵ��ڲ������е�
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

function outPoint = FindLowestThenLeftmostPoint(Points)
%     Point FindLowestThenLeftmostPoint(vector <Point> Points)
%     //�ҵ���ͺ�����ĵ�
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

function outPoint = FindLowestThenRightmostPoint(Points)
% Point FindLowestThenRightmostPoint(vector <Point> Points)
% //�ҵ���ͺ����ҵĵ�
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

function outPoints = PreSorting(Points,LTL)
% vector <Point> PreSorting(vector <Point> Points, Point LTL)

    DeleteRepeatPoint = [];
% 	vector <Point> DeleteRepeatPoint;
    for i=1:size(Points,1)
        if norm(Points(i,:)-LTL)<0.001
            continue;
        end
        DeleteRepeatPoint = [DeleteRepeatPoint;Points(i,:)];
    end

	%//��n��ʱ�临�Ӷ��ж��Ƿ��Ѿ����������
    DeleteRepeatPoint = [LTL;DeleteRepeatPoint];
% 	DeleteRepeatPoint.insert(DeleteRepeatPoint.begin(), LTL);
    if ((size(DeleteRepeatPoint,1)>=2)&&(CheckSorted(DeleteRepeatPoint) == 1))
		test = ToLeftTest(LTL, DeleteRepeatPoint(1,:), DeleteRepeatPoint(2,:));
		i = 2;
        while ((test == 0) && (size(DeleteRepeatPoint,1) >= 3) && (i + 1 <= size(DeleteRepeatPoint,1)))
            test = ToLeftTest(LTL, DeleteRepeatPoint(i,:), DeleteRepeatPoint(i+1,:));
            i = i + 1;
        end
        if (test == -1)
            outPoints = DeleteRepeatPoint;
            return;
        end
    end
    DeleteRepeatPoint(1,:) = [];
% 	DeleteRepeatPoint.erase(DeleteRepeatPoint.begin());

	DeleteRepeatPoint = MergeSortRecur(LTL, DeleteRepeatPoint, 1, size(DeleteRepeatPoint,1));%//�鲢����
    outPoints = DeleteRepeatPoint;
    return;
% 	return DeleteRepeatPoint;
end

function result = CheckSorted(Points)
% int CheckSorted(vector<Point> Points)
% //�ж�Points�Ƿ�Sorted���� ������е㶼�ظ�����ôSorted�ж���Ϊ1 
% //�������ĳ�ʼ����Щ�ظ����Գ�������ܲ�Ӱ�졣
% //O(n)�ĸ��Ӷ�
    if (size(Points,1) < 3)
        result = 1;
        return;
    end
	
% 	vector <Point> Points1; //��ȥ�ظ�Points[0],ֻ����һ��Points[0]�ĵ㼯
    Points1 = [];
    Points1 = [Points1;Points(1,:)];
% 	Points1.push_back(Points[0]);
    for i=2:size(Points,1)
        if norm(Points(i,:)-Points(1,:))>0.001
            Points1 = [Points1;Points(i,:)];
        end
    end
            
	Points = Points1;

% 	int lastcomresult;
    if (size(Points,1) >= 3)
		lastcomresult = ComparePoint(Points(1,:), Points(2,:), Points(3,:));
    end

    for i=3:size(Points,1) - 1
        comresult = ComparePoint(Points(1,:), Points(i,:), Points(i + 1,:));
        if (((lastcomresult == 1 || lastcomresult == 0) && (comresult == 1 || comresult == 0))|| ((lastcomresult == -1 || lastcomresult == 0) && (comresult == -1 || comresult == 0)))
        else
            result = 0;
            return;
% 			return 0;
        end
		lastcomresult = comresult;
    end
    result =1;
end

function [ref,Points,tmp] = Merge(ref,Points,tmp,Left,Right)
% void Merge(Point & ref,vector <Point> & Points, vector<Point> & tmp, int Left, int Right)
% {
    if (Right - Left == 1)
		test = ToLeftTest(ref, Points(Left,:), Points(Right,:));
        if (test == 1)
			temp = Points(Left,:);
			Points(Left,:) = Points(Right,:);
			Points(Right,:) = temp;
        end
		return;
    else
        if (Right == Left)
            return;
        end
    end
	split1Left = Left;
	split1Right = Left + ceil(double((Right - Left + 1) / 2.0)) - 1;
	split2Left = split1Right + 1;
	split2Right = Right;

	[ref,Points,tmp] = Merge(ref,Points, tmp, Left, split1Right);
	[ref,Points,tmp] = Merge(ref,Points, tmp, split2Left, split2Right);

	[ref,Points,tmp] = TwoSplitMerge(ref,Points, tmp, split1Left, split1Right, split2Left, split2Right);
% }
end

function outPoints = MergeSortRecur(ref,Points,Left,Right)
%     vector <Point> MergeSortRecur(Point ref,vector <Point> Points, int Left, int Right)
        if (size(Points,1) <= 1)
            outPoints = Points;
            return;
        end
        tmp = Points;
        [ref,Points,tmp] = Merge(ref,Points, tmp, Left, Right);
        outPoints = Points;
end
    
function result = IfInLine(A,B,C)
% int IfInLine(Point A, Point B,Point C) 
% //�ж�C�ڹ�AB��ɵ�ֱ�ߵĻ������Ƿ������˵���
% //����ڣ���ôreturn 1
% //���� �����AB������֮�� return -1
% //�����BA������֮�⣬return -3
% //�������A����B return -2
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

function out = ComparePoint(A,B1,B2)
% int ComparePoint(Point A, Point B1, Point B2)
% //compare vector(AB1) to vector(AB2) 
% //AB1 greater than AB2 return 1
% //equal return 0
% //smaller return -1
% //Erro return -2
	result = ToLeftTest(A, B1, B2);
    if result == 1
        out = -1;
    else
        if result == -1
            out = 1;
        else
            if result == 0
                out = 0;
            else
                out = -2;
            end
        end
    end
end

function [ref,Points,tmp] = TwoSplitMerge(ref,Points,tmp,split1Left,split1Right,split2Left,split2Right)
% void TwoSplitMerge(Point & ref,vector<Point> & Points, vector<Point> & tmp, int split1Left, int split1Right, int split2Left, int split2Right)
	split1 = split1Left;
	split2 = split2Left;
	tmpzz = 0;
    while (split1 <= split1Right && split2 <= split2Right)
		test = ToLeftTest(ref, Points(split1,:), Points(split2,:));
        if (test == 1)
			tmp(split1Left + tmpzz,:) = Points(split2,:);
			split2 = split2+1;
			tmpzz = tmpzz+1;
        else
            if (test == -1)
                tmp(split1Left + tmpzz,:) = Points(split1,:);
                split1 = split1+1;
                tmpzz = tmpzz+1;
            else
                if (test == 0)
                    tmp(split1Left + tmpzz,:) = Points(split1,:);
                    split1 = split1+1;
                    tmpzz = tmpzz+1;
                    tmp(split1Left + tmpzz,:) = Points(split2,:);
                    split2 = split2+1;
                    tmpzz = tmpzz+1;
                end
            end
        end
    end
    while (split1 <= split1Right)
		tmp(split1Left + tmpzz,:) = Points(split1,:);
		tmpzz = tmpzz+1;
		split1 = split1+1;
    end
    while (split2 <= split2Right)
		tmp(split1Left + tmpzz,:) = Points(split2,:);
		tmpzz = tmpzz+1;
		split2 = split2+1;
    end
	tmpzz = tmpzz-1;
    while (tmpzz >= 0) %//��Ҫ���ںϺ��һ�����з��ص�Data��
		Points(split1Left + tmpzz,:) = tmp(split1Left + tmpzz,:);
		tmpzz = tmpzz-1;
    end
end
