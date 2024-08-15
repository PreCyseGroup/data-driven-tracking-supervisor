function v = partition_index(V,x)

% this function find the index of the voronoi partition 

for i=1:size(V,2)
   if V{i}.contains(x) == 1
       v = i;
   end
end
end

