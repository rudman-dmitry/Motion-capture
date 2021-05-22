function [diff_out] = numeric_diff(mat,Fs,diff_type,window_width,diff_order)

% mat = time x data
% Fs = sample rate in [hz]
% diff_type: 
%       'window' -  nth order derivatives:
%                   (1/h^n)*sum_k=0:n((-1)^(k+n)*(n!/(k!(n-k)!)*f(x+kh))
%       'poly' - polynomial fitting
%       '5points' - 5 points formulation: 
%                       mid : (1/12h) * [f(x-2h)-8f(x-h)+8f(x+h)-f(x+2h)]
%                       end : (1/12h) * [-25f(x)+ 48f(x+h)-36f(x+2h)+16f(x+3h)-3f(x+4h)]
% window_width = number of samples per window
% diff_order = the order of the derivative (1st derivative, 2nd derivative,..)
% diff_out = the derivative of mat in sec (normalized with Fs)
%
% other formulation:
% 2 points formulation: (1/2h)* [f(x+h)-f(x-h)]
% 3 points formulation: (1/2h)* [-3f(x)+4f(x+h)-f(x-2h)]

h = window_width;
n = diff_order;

switch diff_type
    case 'window'
        diff_filter = zeros(1+n*h,1);
        diff_temp = (1/h^n) .* (-1).^(n:2*n) .* (factorial(n)./(factorial(0:n) .* factorial(n:-1:0)));
        diff_temp = flipud(diff_temp.'); % for the convolution
        diff_filter(1:h:end) = diff_temp;
        for col = 1 : size(mat,2)
             diff_out(:,col) = conv(mat(:,col),diff_filter,'valid');
        end
        
    case 'poly'        
        poly_order = 4;
        for col = 1 : size(mat,2)
            for row = 1 : size(mat,1) - window_width
                data_temp = mat(row : row + window_width-1,col);
                t = (0 : 1/Fs : (length(data_temp)-1)/Fs).';
                p = polyfit(t,data_temp,poly_order);
                for k = 1 : n
                    p = polyder(p);
                end
                diff_out(row,col) = polyval(p,t(round(end/2)));
            end
        end        
        
    case '5points'
        diff_filter_mid = (1/12*h)*[-1,8,0,-8,1].';
        diff_filter_end = (1/12*h)*[-25,48,-36,16,-3].';
        for col = 1 : size(mat,2)
             diff_mid = conv(mat(:,col),diff_filter_mid,'valid');
             diff_left =  conv(mat(1:2+4,col),diff_filter_end,'valid');
             diff_right = conv(mat((end-2+1)-4:end,col),diff_filter_end,'valid');
             if size(mat,1)>2
                diff_out_temp = [diff_left; diff_mid; diff_right];
            else 
                disp("matrix contains only two time points per each data feature");
            end
            diff_out(:,col) = diff_out_temp;
        end
        
    otherwise
        disp("choose the foloowing diff type: 'window','poly'...");
end
diff_out = diff_out * Fs^n; % [.]/sec