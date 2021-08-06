%%
%subplot
figure(10);
subplot(4,1,1);
grid on;
    hold on;
    plot( Y_pitch.Time, Y_pitch.Data(:,1)+norm(Xe(1:3)),'-','LineWidth', 1.5);
    plot( NLY_pitch.Time, NLY_pitch.Data(:,1),'-.','LineWidth', 1.5);
    legend('Resposta Linear', 'Resposta Não-Linear');
    ylabel('V_{T} (m/s)');
    ylim([17 24]);
subplot(4,1,2);
    grid on;
    hold on;
    plot( Y_pitch.Time, radtodeg(Y_pitch.Data(:,2)+Xe(8)), '-','LineWidth', 1.5);
    plot( NLY_pitch.Time, radtodeg(NLY_pitch.Data(:,2)), '-.','LineWidth', 1.5);
    ylabel('\alpha (º)');
    ylim([1.5 4.5]);
subplot(4,1,3);
    grid on;
    hold on;
    plot( Y_pitch.Time, radtodeg(Y_pitch.Data(:,3)), '-','LineWidth', 1.5);
    plot( NLY_pitch.Time, radtodeg(NLY_pitch.Data(:,3)), '-.','LineWidth', 1.5);
    ylabel('q (º/s)'); 
    ylim([-5 20]);
subplot(4,1,4);
    grid on;
    hold on;
    plot( Y_pitch.Time, Y_pitch.Data(:,5)-Xe(12), '-','LineWidth', 1.5);
    plot( NLY_pitch.Time, NLY_pitch.Data(:,5), '-.','LineWidth', 1.5);
    ylabel('h (m)');
    xlabel('Tempo (s)');
    ylim([400 500]);     
samexaxis('ytac','xmt','off', 'yld', 1)
%%