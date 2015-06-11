function q = convertQ(q, reverse)
    toNeg = 11:13;
    if nargin >=2 && reverse
        q(toNeg,:) = -q(toNeg,:);
        q = q([1 2 3 4 5 6 7 8 9],:);   
    else
        q = q([1 2 3 4 5 6 7 8 9 9 6 5 4],:);
        q(toNeg,:) = -q(toNeg,:);
    end
end