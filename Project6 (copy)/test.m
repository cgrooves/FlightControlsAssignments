x = [0 0 0 P.Va0 0 0 0 gamma 0 0 0 0]';
u = [0 0 0 1]';
t = 0;
fcn = @mavsim_trim;
mavsim_trim([],[],[],'compile')
y = feval(fcn,t,x,u,'outputs')