function R1 = TS_Init(x,u,AB,W)


x1 = zonotope(x,0*diag(ones(2,1)));
u1 = zonotope(u,0*diag(ones(2,1)));
R1 = (AB * (cartProd(x1,u1))) + W;

end

