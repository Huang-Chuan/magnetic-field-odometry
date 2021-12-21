function error_struct = calc_error_2(XData, PData, Xs, numSamples, N)
%   error sturct
    error_struct = struct('msePosErr_x',     [], 'stdPosErr_x', [], ...
                          'mseVelErr_x',     [], 'stdVelErr_x', [], ...
                          'mseVelErr_y',     [], 'stdVelErr_y', [], ...
                          'mseVelErr_z',     [], 'stdVelErr_z', [], ...
                          'mseAccBiasErr_x', [], 'stdAccBiasErr_x', [], ... 
                          'mseAccBiasErr_y', [], 'stdAccBiasErr_y', [], ... 
                          'mseAccBiasErr_z', [], 'stdAccBiasErr_z', [], ... 
                          'mseMagBiasErr', [], 'stdMagBiasErr', [], ...
                          'mseC2Err',      [], 'stdC2Err',      [], ...
                          'mseC1Err',      [], 'stdC1Err',      [], ...
                          'mseC0Err',      [], 'stdC0Err',      []);
    % position 
    msePosErr_x      = zeros(numSamples, N);
    stdPosErr_x      = zeros(numSamples, N);
    msePosErr_y      = zeros(numSamples, N);
    stdPosErr_y      = zeros(numSamples, N);
    msePosErr_z      = zeros(numSamples, N);
    stdPosErr_z      = zeros(numSamples, N);



    % velocity
    mseVelErr_x      = zeros(numSamples, N);
    stdVelErr_x      = zeros(numSamples, N);
    mseVelErr_y      = zeros(numSamples, N);
    stdVelErr_y      = zeros(numSamples, N);
    mseVelErr_z      = zeros(numSamples, N);
    stdVelErr_z      = zeros(numSamples, N);
    
    mseAccBiasErr_x     = zeros(numSamples, N);
    stdAccBiasErr_x     = zeros(numSamples, N);
    mseAccBiasErr_y     = zeros(numSamples, N);
    stdAccBiasErr_y     = zeros(numSamples, N);
    mseAccBiasErr_z     = zeros(numSamples, N);
    stdAccBiasErr_z     = zeros(numSamples, N);
    % bias in 3 sensors
    % mseMagBiasErr1    = zeros(numSamples, N);
    % stdMagBiasErr1     = zeros(numSamples, N);


    % % parameter error
    % mseC2Err          = zeros(numSamples, N);
    % stdC2Err          = zeros(numSamples, N);
    % mseC1Err          = zeros(numSamples, N);
    % stdC1Err          = zeros(numSamples, N);
    % mseC0Err          = zeros(numSamples, N);
    % stdC0Err          = zeros(numSamples, N);

    % for iter = 1 : N
    %     mseError(:, iter) = (XData{iter} - Xs{iter}).^2;
    %     stdError(:, iter) = 
    % end
    for iter = 1 : N
        
        msePosErr_x(:, iter) = (XData{iter}(:, 1) - Xs{iter}(:, 1)).^2;
        stdPosErr_x(:, iter) = PData{iter}(:, 1);
        
        mseVelErr_x(:, iter) = (XData{iter}(:, 4) - Xs{iter}(:, 4)).^2;
        stdVelErr_x(:, iter) = PData{iter}(:, 4);

        mseVelErr_y(:, iter) = (XData{iter}(:, 5) - Xs{iter}(:, 5)).^2;
        stdVelErr_y(:, iter) = PData{iter}(:, 5);

        mseVelErr_z(:, iter) = (XData{iter}(:, 6) - Xs{iter}(:, 6)).^2;
        stdVelErr_z(:, iter) = PData{iter}(:, 6);

        % mseC2Err(:, iter) = (XData{iter}(:, end - 2) - Xs{iter}(:, end - 2)).^2;
        % stdC2Err(:, iter) = PData{iter}(:, end - 2, end - 2);
        % mseC1Err(:, iter) = (XData{iter}(:, end - 1) - Xs{iter}(:, end - 1)).^2;
        % stdC1Err(:, iter) = PData{iter}(:, end - 1, end - 1);
        % mseC0Err(:, iter) = (XData{iter}(:, end) - Xs{iter}(:, end)).^2;
        % stdC0Err(:, iter) = PData{iter}(:, end, end);
        % if(opt.hasAccBias) && (opt.hasMagBias) 
        mseAccBiasErr_x(:, iter) = (XData{iter}(:, 7) - Xs{iter}(:, 7)).^2;
        stdAccBiasErr_x(:, iter)= PData{iter}(:, 7);
    
        %     mseMagBiasErr1(:, iter)    = (XData{iter}(:, 4) - Xs{iter}(:, 4)).^2;
        %     stdMagBiasErr1(:, iter)     = PData{iter}(:, 4, 4);
        % elseif(opt.hasAccBias) && (~opt.hasMagBias) 
        %     mseAccBiasErr(:, iter) = (XData{iter}(:, 3) - Xs{iter}(:, 3)).^2;
        %     stdAccBiasErr(:, iter)= PData{iter}(:, 3, 3);
        % elseif (~opt.hasAccBias) && (opt.hasMagBias) 
        %     mseMagBiasErr1(:, iter)    = (XData{iter}(:, 3) - Xs{iter}(:, 3)).^2;
        %     stdMagBiasErr1(:, iter)    = PData{iter}(:, 3, 3);   
        % end
        

    end
    
    
    error_struct.msePosErr_x = sqrt(mean(msePosErr_x, 2));
    error_struct.stdPosErr_x = sqrt(mean(stdPosErr_x, 2));
    
    error_struct.mseVelErr_x = sqrt(mean(mseVelErr_x, 2));
    error_struct.stdVelErr_x = sqrt(mean(stdVelErr_x, 2));
    error_struct.mseVelErr_y = sqrt(mean(mseVelErr_y, 2));
    error_struct.stdVelErr_y = sqrt(mean(stdVelErr_y, 2));
    error_struct.mseVelErr_z = sqrt(mean(mseVelErr_z, 2));
    error_struct.stdVelErr_z = sqrt(mean(stdVelErr_z, 2));


    error_struct.mseAccBiasErr_x = sqrt(mean(mseAccBiasErr_x, 2));
    error_struct.stdAccBiasErr_x = sqrt(mean(stdAccBiasErr_x, 2));
    

    
    % error_struct.stdC2Err = sqrt(mean(stdC2Err, 2));
    % error_struct.mseC2Err = sqrt(mean(mseC2Err, 2));
    % error_struct.stdC1Err = sqrt(mean(stdC1Err, 2));
    % error_struct.mseC1Err = sqrt(mean(mseC1Err, 2));
    % error_struct.stdC0Err = sqrt(mean(stdC0Err, 2));
    % error_struct.mseC0Err = sqrt(mean(mseC0Err, 2));

end
