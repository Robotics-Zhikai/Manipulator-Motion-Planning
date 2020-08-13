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
