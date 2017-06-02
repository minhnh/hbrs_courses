%% Rosenbrock function
function f=frosen(x)
  if size(x,1) < 2; error('dimension must be greater one'); end
  f = 1e2*sum((x(1:end-1,:).^2 - x(2:end,:)).^2,1) + sum((x(1:end-1,:)-1).^2,1);
end