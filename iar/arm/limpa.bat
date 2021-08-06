@echo off
REM ==================================================================================
REM === Apaga todos os arquivos desnecessários do diretório "pegasus\arm" e seus   ===
REM === subdiretórios.                                                             ===
REM ==================================================================================
REM === Verifica se é o Windows NT ou superior. Se for, executa o comando "del".   ===
REM === Se não for, assume que é o Windows 95/98/ME e executa o comando "deltree". ===
REM ==================================================================================

REM === "SetLocal" evita que as variáveis de ambiente do sistema sejam alteradas ===
SetLocal

REM === Arquivos a serem apagados ===
Set FILES= *.bin *.out *.sim *.map *.o *.pbd *.pbi

if %OS%==Windows_NT (
    del /s  /q %FILES%
) else (
    deltree /y %FILES%
)
