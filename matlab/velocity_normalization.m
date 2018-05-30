function y = velocity_normalization(x, interval_x, interval_y)
% y = velocity_normalization(x, interval_x, interval_y)
%
% Normalize the input velocity x between the input interval (interval_x,
% [min_x max_x]) and the provided interval (interval_y).
% interval_y can be in the format [min_y max_y] or 
% [max_negative_ycmin_negative_y min_positive_y max_positive_y]


    min_abs_x = interval_x(1);
    max_abs_x = interval_x(2);

    min_abs_y = interval_y(1);
    max_abs_y = interval_y(2);
    

    y = zeros(size(x));
    for i = 1:length(x)
        cx = x(i);

        sign = 1;
        if (cx < 0)
            sign = -1;
        end
        
        
%         if (cx > max_abs_x)
%             cx = max_abs_x;
%         elseif(cx < -max_abs_x)
%             cx = -max_abs_x;
%         end
        
        ax = abs(cx);
        
        if(ax > max_abs_x)
            ax = max_abs_x;
        end
       
        maxx = max_abs_x;
        minx = min_abs_x;
        b = max_abs_y;
        a = min_abs_y;
        y(i) = sign*((b - a).*((ax - minx)./(maxx - minx)) + a);
        
    end


end