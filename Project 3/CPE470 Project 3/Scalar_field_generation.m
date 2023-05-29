% For CPE 470/670
% By Dr. Hung (Jim) La @ 2022 
%====PARAMETER OF GAUSSIAN MODEL (MULTIVARIATE NORMAL DISTRIBUTION)===
x_neg = -6;
x_pos = 6;
y_neg = -6;
y_pos = 6;

%Gaussian distribution 0
mu_x1 = 3;%2
mu_y1 = 2;%0
mu1 = [mu_x1 mu_y1];
variance_x1 = 2.25;
variance_y1 = 2.25;
rho1 = 0.1333; %correlation between 2 variabes (#1)
covariance_xy1 = rho1*sqrt(variance_x1*variance_y1);
%Sigma = 0.25*[.25 .3; .3 1];
Sigma1 = [variance_x1 covariance_xy1; covariance_xy1 variance_y1];

mu_x2 = 1;%0;
mu_y2 = 4.5;%2
mu2 = [mu_x2 mu_y2];
variance_x2 = 1.25;
variance_y2 = 1.25;
rho2 = 0.1333; %correlation between 2 variabes (#1)
covariance_xy2 = rho2*sqrt(variance_x2*variance_y2);
%Sigma = 0.25*[.25 .3; .3 1];
Sigma2 = [variance_x2 covariance_xy2; covariance_xy2 variance_y2];

mu_x3 = -2;%0;
mu_y3 = 3;%2
mu3 = [mu_x3 mu_y3];
variance_x3 = 1.25;
variance_y3 = 1.25;
rho3 = 0.1333; %correlation between 2 variabes (#1)
covariance_xy3 = rho3*sqrt(variance_x3*variance_y3);
%Sigma = 0.25*[.25 .3; .3 1];
Sigma3 = [variance_x3 covariance_xy3; covariance_xy3 variance_y3];

mu_x4 = 4;%0;
mu_y4 = -4;%2
mu4 = [mu_x4 mu_y4];
variance_x4 = 1.25;
variance_y4 = 1.25;
rho4 = 0.1333; %correlation between 2 variabes (#1)
covariance_xy4 = rho4*sqrt(variance_x4*variance_y4);
%Sigma = 0.25*[.25 .3; .3 1];
Sigma4 = [variance_x4 covariance_xy4; covariance_xy4 variance_y4];


%Devide the Region F into cells
scal = 0.5;%0.1;
x = x_neg:scal:x_pos; %x dimension of the survellance Region
y = y_neg:scal:y_pos; %y dimension of the survellance Region
[X1,X2] = meshgrid(x,y);
Theta = [30 10 8 20]; %The true constant vector

Phi = [mvnpdf([X1(:) X2(:)],mu1,Sigma1) mvnpdf([X1(:) X2(:)],mu2,Sigma2) mvnpdf([X1(:) X2(:)],mu3,Sigma3) mvnpdf([X1(:) X2(:)],mu4,Sigma4)]; %The distribution of the environment

F1 = Theta*Phi';

num_cells = length(F1);
F = reshape(F1,length(y),length(x)); %Find cell's value (y by x matrix)
%F =[F2(2,:); F2(1,:)];
figure(1), surf(-F) %Plot the true map