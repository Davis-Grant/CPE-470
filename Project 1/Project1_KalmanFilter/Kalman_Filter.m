function [s] = Kalman_Filter(s)
Ak = s.A * s.x;
Bk = s.A * s.P * transpose(s.A) + s.Q;
Ck = Bk * s.H / (s.H * Bk * transpose(s.H) + s.R);
s.x = Ak + Ck * (s.z - (s.H * Ak));
s.P = Bk - (Ck * s.H * Bk);
end