function outPoints = PreSorting(Points,LTL)
% vector <Point> PreSorting(vector <Point> Points, Point LTL)

    DeleteRepeatPoint = [];
% 	vector <Point> DeleteRepeatPoint;
    for i=1:size(Points,1)
        if norm(Points(i,:)-LTL)==0 || norm(Points(i,:)-LTL)<1e-15
            continue;
        end
        DeleteRepeatPoint = [DeleteRepeatPoint;Points(i,:)];
    end
    
    if isempty(DeleteRepeatPoint)==1
        DeleteRepeatPoint = Points(2:end,:);
    end

	%//用n的时间复杂度判断是否已经是排序完成
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
            DeleteRepeatPoint(1,:) = [];
            outPoints = DeleteRepeatPoint;
            return;
        end
    end
    DeleteRepeatPoint(1,:) = [];
% 	DeleteRepeatPoint.erase(DeleteRepeatPoint.begin());

	DeleteRepeatPoint = MergeSortRecur(LTL, DeleteRepeatPoint, 1, size(DeleteRepeatPoint,1));%//归并排序
    outPoints = DeleteRepeatPoint;
    return;
% 	return DeleteRepeatPoint;
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
    while (tmpzz >= 0) %//需要把融合后的一段序列返回到Data中
		Points(split1Left + tmpzz,:) = tmp(split1Left + tmpzz,:);
		tmpzz = tmpzz-1;
    end
end

