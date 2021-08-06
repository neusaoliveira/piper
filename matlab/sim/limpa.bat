@echo off
REM ==================================================================================
REM === Apaga todos os arquivos desnecess�rios do diret�rio e seus subdiret�rios.  ===                                                            
REM ==================================================================================
REM === Verifica se � o Windows NT ou superior. Se for, executa o comando "del".   ===
REM === Se n�o for, assume que � o Windows 95/98/ME e executa o comando "deltree". ===
REM ==================================================================================

REM === "SetLocal" evita que as vari�veis de ambiente do sistema sejam alteradas   ===
SetLocal

REM === Arquivos a serem apagados ===
Set FILES= *.c *.mexw64 *.tlc 

if %OS%==Windows_NT (
    del /s  /q %FILES%
) else (
    deltree /y %FILES%
)
