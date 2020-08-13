classdef Segment
    properties
        L = [0 0 0];
        R = [0 0 0];
    end
    methods
        function obj = Segment(A,B)
            if A(1) > B(1)
                obj.L = B;
                obj.R = A;
            else
                if A(1) == B(1)
                    if A(2)<=B(2)
                        obj.L = A;
                        obj.R = B;
                    else
                        obj.L = B;
                        obj.R = A;
                    end
                else
                    obj.L = A;
                    obj.R = B;
                end
            end
        end
        
    end
end
