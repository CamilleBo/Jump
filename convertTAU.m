function TAU = convertTAU(TAU, reverse)
    toNeg = 8:10;
    if nargin >=2 && reverse
        TAU(toNeg) = -TAU(toNeg);
        TAU = TAU([1 2 3 4 5 6     ],:);
    else
        TAU = TAU([1 2 3 4 5 6 6 3 2 1],:);
        TAU(toNeg) = -TAU(toNeg);
    end
    

end