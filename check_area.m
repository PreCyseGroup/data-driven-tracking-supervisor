function  out = check_area(index_r,index_x, I)

j = 1;
for i=1:5
    if I{i,index_r} > I{index_x,index_r}
        out(j) = i;
        j = j+1;
    end
end
end

