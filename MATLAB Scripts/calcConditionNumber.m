% Interesting: harder to control the shorter the ramp becomes
% (the condition number goes up)

function calcConditionNumber()
% Define plausible values for system parameters
% Source of moment of inertia equations:
% https://en.wikipedia.org/wiki/List_of_moments_of_inertia

[A, B, C, D] =  formSystem();

% Form the observability matrix
obsMat = [C; C*A; C*A^2; C*A^3];

% Display observability matrix, along with its rank
obsMat
X = ['Rank: ', num2str(rank(obsMat))];
disp(X)

% Display condition num
X = ['Condition number: ', num2str(cond(obsMat))];
disp(X)

% Form observability  matrix if we only observe position
Cpos = [1, 0, 0, 0];
obsMatPos = [Cpos; Cpos*A; Cpos*A^2; Cpos*A^3];
obsMatPos
X = ['Rank: ', num2str(rank(obsMatPos))];
disp(X)
X = ['Condition number: ', num2str(cond(obsMatPos))];
disp(X)

   
% Form observability  matrix if we only observe angle
% Cang = [0, 0, 1, 0];
% obsMatAng = [Cang; Cang*A; Cang*A^2; Cang*A^3];
% obsMatAng
% X = ['Rank: ', num2str(rank(obsMatAng))];
% disp(X)
% X = ['Condition number: ', num2str(cond(obsMatAng))];
% disp(X)
% 
% 
% % Form the controllability matrix
% contMat = [B, A*B, A^2*B, A^3*B];
% 
% % Display controllability matrix, along with its rank
% contMat
% X = ['Rank: ', num2str(rank(contMat))];
% disp(X)
% 
% 
% % Display condition number
% % Larger than 10 - may indicate control problmes
% X = ['Condition number: ', num2str(cond(contMat))];
% disp(X)







