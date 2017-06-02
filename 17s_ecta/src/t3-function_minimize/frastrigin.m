%% Rastrigin function
function f=frastrigin(x)
  N = size(x,1);
  f = (N/20)*807.06580387678 - (10 * (N-sum(cos(2*pi*x),1)) + sum(x.^2,1));
  f(any(abs(x) > 5.12)) = 1e2*N;
  f = -f;
end
