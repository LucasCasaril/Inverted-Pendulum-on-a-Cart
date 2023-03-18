function dx = dynamical_system(x,m,M,l,g,u)

Sx = sin(x(3));
Cx = cos(x(3));
den = (M + m*(1-Cx^2));

dx(1,1) = x(2);
dx(2,1) = (1/den)*(g*m*Cx*Sx - m*l*(x(4)^2)*Sx) + (1/den)*u;
dx(3,1) = x(4);
dx(4,1) = (1/(l*den))*(g*Sx*(M + m) - m*l*(x(4)^2)*Sx*Cx) + (1/(l*den))*u*Cx;