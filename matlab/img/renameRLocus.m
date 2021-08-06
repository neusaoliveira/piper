%%
% Rename rlocus plot
axIm = findall(gcf,'String','Imaginary Axis (seconds^{-1})');
axRe = findall(gcf,'String','Real Axis (seconds^{-1})');

title ('Lugar Geométrico das Raízes');
set(axIm,'String','Eixo Imaginário (1/s)');
set(axRe,'String','Eixo Real (1/s)');

clear axIm axRe
%%