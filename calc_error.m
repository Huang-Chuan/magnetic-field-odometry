function error_struct = calc_error(XData, PData, pos, vel, accbias, magbias, c, numSamples, N, opt)
%   error sturct
    error_struct = struct('msePosErr',     [], 'stdPosErr', [], ...
                          'mseVelErr',     [], 'stdVelErr', [], ...
                          'mseAccBiasErr', [], 'stdAccBiasErr', [], ... 
                          'mseMagBiasErr', [], 'stdMagBiasErr', [], ...
                          'mseC2Err',      [], 'stdC2Err',      [], ...
                          'mseC1Err',      [], 'stdC1Err',      [], ...
                          'mseC0Err',      [], 'stdC0Err',      []);
  
                      
    msePosErr      = zeros(numSamples, N);
    stdPosErr      = zeros(numSamples, N);
    mseVelErr      = zeros(numSamples, N);
    stdVelErr      = zeros(numSamples, N);
    
    
    mseAccBiasErr     = zeros(numSamples, N);
    stdAccBiasErr     = zeros(numSamples, N);
    
    % bias in 3 sensors
    mseMagBiasErr1    = zeros(numSamples, N);
    stdMagBiasErr1     = zeros(numSamples, N);


    % parameter error
    mseC2Err          = zeros(numSamples, N);
    stdC2Err          = zeros(numSamples, N);
    mseC1Err          = zeros(numSamples, N);
    stdC1Err          = zeros(numSamples, N);
    mseC0Err          = zeros(numSamples, N);
    stdC0Err          = zeros(numSamples, N);

    for iter = 1 : N
        
        msePosErr(:, iter) = (XData{iter}(:, 1) - pos(:, 1)).^2;
        stdPosErr(:, iter) = PData{iter}(:, 1, 1);
        
        mseVelErr(:, iter) = (XData{iter}(:, 2) - vel(:, 1)).^2;
        stdVelErr(:, iter) = PData{iter}(:, 2, 2);


        mseC2Err(:, iter) = (XData{iter}(:, end - 2) - c(:, 1)).^2;
        stdC2Err(:, iter) = PData{iter}(:, end - 2, end - 2);
        mseC1Err(:, iter) = (XData{iter}(:, end - 1) - c(:, 2)).^2;
        stdC1Err(:, iter) = PData{iter}(:, end - 1, end - 1);
        mseC0Err(:, iter) = (XData{iter}(:, end) - c(:, 3)).^2;
        stdC0Err(:, iter) = PData{iter}(:, end, end);

    end
    
    
    if(opt.hasAccBias) && (opt.hasMagBias) 
        mseAccBiasErr(:, iter) = (XData{iter}(:, 3) - accbias(1, iter)).^2;
        stdAccBiasErr(:, iter)= PData{iter}(:, 3, 3);

        mseMagBiasErr1    = (XData{iter}(:, 4) - magbias(1, iter)).^2;
        stdMagBiasErr1     = PData{iter}(:, 4, 4);
    elseif(opt.hasAccBias) && (~opt.hasMagBias) 
        mseAccBiasErr(:, iter) = (XData{iter}(:, 3) - accbias(1, iter)).^2;
        stdAccBiasErr(:, iter)= PData{iter}(:, 3, 3);
    elseif (~opt.hasAccBias) && (opt.hasMagBias) 
        mseMagBiasErr1    = (XData{iter}(:, 3) - magbias(1, iter)).^2;
        stdMagBiasErr1     = PData{iter}(:, 3, 3);   
    end
    
    error_struct.msePosErr = sqrt(mean(msePosErr, 2));
    error_struct.stdPosErr = sqrt(mean(stdPosErr, 2));
    
    error_struct.mseVelErr = sqrt(mean(mseVelErr, 2));
    error_struct.stdVelErr = sqrt(mean(stdVelErr, 2));
    if(opt.hasAccBias)
        error_struct.mseAccBiasErr = sqrt(mean(mseAccBiasErr, 2));
        error_struct.stdAccBiasErr = sqrt(mean(stdAccBiasErr, 2));
    end
    
    if(opt.hasMagBias)
        error_struct.mseMagBiasErr = sqrt(mean(mseMagBiasErr1, 2));
        error_struct.stdMagBiasErr = sqrt(mean(stdMagBiasErr1, 2)); 
    end
    
    error_struct.stdC2Err = sqrt(mean(stdC2Err, 2));
    error_struct.mseC2Err = sqrt(mean(mseC2Err, 2));
    error_struct.stdC1Err = sqrt(mean(stdC1Err, 2));
    error_struct.mseC1Err = sqrt(mean(mseC1Err, 2));
    error_struct.stdC0Err = sqrt(mean(stdC0Err, 2));
    error_struct.mseC0Err = sqrt(mean(mseC0Err, 2));

end

