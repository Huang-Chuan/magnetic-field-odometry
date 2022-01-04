    figure;
    subplot(3,1,1)
    hold on;
    curve1 =  error_struct.stdPosErr(:, 1);
    curve2 =  zeros(size(error_struct.stdPosErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.msePosErr(:, 1), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('position error x');
   
    %saveas(gcf,sprintf('exp%d\\PosError.jpg', 0));

    subplot(3,1,2)
    hold on;
    curve1 =  error_struct.stdPosErr(:, 2);
    curve2 =  zeros(size(error_struct.stdPosErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.msePosErr(:, 2), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('position error y');
    %


    subplot(3,1,3)
    hold on;
    curve1 =  error_struct.stdPosErr(:, 3);
    curve2 =  zeros(size(error_struct.stdPosErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.msePosErr(:, 3), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m');
    title('position error z');
    %



    % % velocity error plot
    figure;
    subplot(3,1,1)
    hold on;
    curve1 =  error_struct.stdVelErr(:, 1);
    curve2 =  zeros(size(error_struct.stdVelErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error x');
    
    %saveas(gcf,sprintf('exp%d\\VelErrorX.jpg', 0));

    subplot(3,1,2)
    hold on;
    curve1 =  error_struct.stdVelErr(:, 2);
    curve2 =  zeros(size(error_struct.stdVelErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error y');
    
    %saveas(gcf,sprintf('exp%d\\VelErrorY.jpg', 0));


    subplot(3,1,3)
    hold on;
    curve1 =  error_struct.stdVelErr(:, 3);
    curve2 =  zeros(size(error_struct.stdVelErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseVelErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-1}');
    title('velocity error z');
    
   %saveas(gcf,sprintf('exp%d\\VelErrorZ.jpg', 0));


    % orientation error
    figure;
    subplot(3,1,1)
    hold on;
    curve1 =  error_struct.stdQErr(:, 1);
    curve2 =  zeros(size(error_struct.stdQErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, 2 * error_struct.mseQErr(:, 1), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/rad');
    title('orientation error x');
    %
    %saveas(gcf,sprintf('exp%d\\PosError.jpg', 0));

    subplot(3,1,2)
    hold on;
    curve1 =  error_struct.stdQErr(:, 2);
    curve2 =  zeros(size(error_struct.stdQErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, 2 * error_struct.mseQErr(:, 2), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/rad');
    title('orientation error y');
    %


    subplot(3,1,3)
    hold on;
    curve1 =  error_struct.stdQErr(:, 3);
    curve2 =  zeros(size(error_struct.stdQErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma Upd');
    h.FaceAlpha = 0.8;
    plot(timeVector, 2 * error_struct.mseQErr(:, 3), 'r','LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/rad');
    title('orientation error z');
   

   
    % % acc bias error plot
    % if(opt.hasAccBias)
    figure;
    subplot(3,1,1)
    hold on;
    curve1 =  error_struct.stdAccBiasErr(:, 1);
    curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseAccBiasErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('acc bias error x');
    
    %saveas(gcf, sprintf('exp%d\\accBiasError.jpg', 0));
    subplot(3,1,2)
    hold on;
    curve1 =  error_struct.stdAccBiasErr(:, 2);
    curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseAccBiasErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('acc bias error y');
    

    subplot(3,1,3)
    hold on;
    curve1 =  error_struct.stdAccBiasErr(:, 3);
    curve2 =  zeros(size(error_struct.stdAccBiasErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseAccBiasErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/m.s^{-2}');
    title('acc bias error z');
    


    figure;
    subplot(3,1,1)
    hold on;
    curve1 =  error_struct.stdMagBiasErr(:, 1);
    curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseMagBiasErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/ \mu T');
    title('magnetometer bias error x'); 
    
    %saveas(gcf, sprintf('exp%d\\magBiasError.jpg', 0));
    subplot(3,1,2)
    hold on;
    curve1 =  error_struct.stdMagBiasErr(:, 2);
    curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseMagBiasErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/ \mu T');
    title('magnetometer bias error y'); 
    
    
    subplot(3,1,3)
    hold on;
    curve1 =  error_struct.stdMagBiasErr(:, 3);
    curve2 =  zeros(size(error_struct.stdMagBiasErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseMagBiasErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/ \mu T');
    title('magnetometer bias error z'); 
    


    % gyro bias
    figure;
    subplot(3,1,1)
    hold on;
    curve1 =  error_struct.stdGyroErr(:, 1);
    curve2 =  zeros(size(error_struct.stdGyroErr(:, 1)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseGyroErr(:, 1), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/rad');
    title('gyro bias error x');
    
    %saveas(gcf,sprintf('exp%d\\VelErrorX.jpg', 0));

    subplot(3,1,2)
    hold on;
    curve1 =  error_struct.stdGyroErr(:, 2);
    curve2 =  zeros(size(error_struct.stdGyroErr(:, 2)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseGyroErr(:, 2), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/rad');
    title('gyro bias error y');
    
    %saveas(gcf,sprintf('exp%d\\VelErrorY.jpg', 0));


    subplot(3,1,3)
    hold on;
    curve1 =  error_struct.stdGyroErr(:, 3);
    curve2 =  zeros(size(error_struct.stdGyroErr(:, 3)));
    inBetween = [curve1; flipud(curve2)];
    x = [timeVector fliplr(timeVector)];
    h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    h.FaceAlpha = 0.8;
    plot(timeVector, error_struct.mseGyroErr(:, 3), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    xlabel('time/s');
    ylabel('error/rad');
    title('gyro bias error z');
    
    
    
    % coefficient plot
    figure;
    for i = 1 : 15
        subplot(3,5,i)
        hold on;
        curve1 =  error_struct.stdThetaErr(:, i);
        curve2 =  zeros(size(error_struct.stdThetaErr(:, i)));
        inBetween = [curve1; flipud(curve2)];
        x = [timeVector fliplr(timeVector)];
        h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
        h.FaceAlpha = 0.8;
        plot(timeVector, error_struct.mseThetaErr(:, i), 'r', 'LineWidth', 2, 'DisplayName','Upd');
        xlabel('time/s');
        ylabel('error');
        title(sprintf('theta %d error', i));
    end
    %saveas(gcf,sprintf('exp%d\\VelErrorX.jpg', 0));

    % subplot(3,1,2)
    % hold on;
    % curve1 =  error_struct.stdThetaErr(:, 5);
    % curve2 =  zeros(size(error_struct.stdThetaErr(:, 5)));
    % inBetween = [curve1; flipud(curve2)];
    % x = [timeVector fliplr(timeVector)];
    % h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    % h.FaceAlpha = 0.8;
    % plot(timeVector, error_struct.mseThetaErr(:, 5), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    % xlabel('time/s');
    % ylabel('error');
    % title('theta 5 error');
    
    % %saveas(gcf,sprintf('exp%d\\VelErrorY.jpg', 0));


    % subplot(3,1,3)
    % hold on;
    % curve1 =  error_struct.stdThetaErr(:, 10);
    % curve2 =  zeros(size(error_struct.stdThetaErr(:, 10)));
    % inBetween = [curve1; flipud(curve2)];
    % x = [timeVector fliplr(timeVector)];
    % h = fill(x, inBetween, [0.9882 0.4157 0.9020], 'DisplayName','\sigma');
    % h.FaceAlpha = 0.8;
    % plot(timeVector, error_struct.mseThetaErr(:, 10), 'r', 'LineWidth', 2, 'DisplayName','Upd');
    % xlabel('time/s');
    % ylabel('error');
    % title('theta 10 error');
    
    
    
    