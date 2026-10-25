function bites=rad2bites(rad) 

constants = [198.625 512; 185.9 512; 185.9 512; 196.08 820];

for i=1:size(rad,1)
    bites(i,:) = constants(i,1).*rad(i,:) + constants(i,2);
end

end