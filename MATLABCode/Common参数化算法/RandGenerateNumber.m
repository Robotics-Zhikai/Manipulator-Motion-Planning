function RandNumber = RandGenerateNumber(a,b,Num)
    RandNumber = a + (b-a).*rand(Num,1);
end
