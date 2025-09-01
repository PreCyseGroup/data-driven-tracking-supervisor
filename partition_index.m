function v = partition_index(V,x)

% This function find the index of the Voronoi partition 

for i=1:size(V,2)
   if V{i}.contains(x) == 1
       v = i;
   end
end
end


