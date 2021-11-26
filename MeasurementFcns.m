classdef MeasurementFcns
    properties 
        opt
    end
    methods(Static)
        function [yk] = MeasurementFcn0(x, sensor_disp)
            r = sensor_disp;
            yk = [r^2 -r 1; 0 0 1; r^2 r 1] * x(end-2:end);
        end

        function [yk] = MeasurementFcn1(x, sensor_disp)
            r = sensor_disp;
            yk = [r^2 -r 1; 0 0 1; r^2 r 1] * x(end-2:end);
        end

        function [yk] = MeasurementFcn2(x, sensor_disp)
            r = sensor_disp;
            yk = [r^2 -r 1; 0 0 1; r^2 r 1] * x(end-2:end) + x(3:5);
        end

        function [yk] = MeasurementFcn3(x, sensor_disp)
            r = sensor_disp;
            yk = [r^2 -r 1; 0 0 1; r^2 r 1] * x(end-2:end) + x(4:6);
        end

    end

    methods
        function [MeasurementFcn] = getMeasurementFcn(obj)
            if (~obj.opt.hasMagBias) && (~obj.opt.hasAccBias) 
                MeasurementFcn = @obj.MeasurementFcn0;
            elseif(~obj.opt.hasMagBias) && (obj.opt.hasAccBias) 
                MeasurementFcn = @obj.MeasurementFcn1;
            elseif(obj.opt.hasMagBias) && (~obj.opt.hasAccBias) 
                MeasurementFcn = @obj.MeasurementFcn2;
            elseif(obj.opt.hasMagBias) && (obj.opt.hasAccBias) 
                MeasurementFcn = @obj.MeasurementFcn3;
            else
               MeasurementFcn = 0;
            end
         end
    
    
    end


end