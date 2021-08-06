<h1 align="center">
    <a href="#"> Design and Bench Testing of an Autopilot for Unmanned Air Vehicles  </a>
</h1>

## Software Requirements

- MATLAB R2016a with MinGW-w64 Compiler installed
- IAR Embedded Workbench IDE

## Hardware Requirements

- Stellaris EKT-LM3S6965 

## Running Simulink Simulation

1. Open the folder `\matlab` in MATLAB.
2. Open the file `executar.m`.

    The file `executar.m` contains the waypoints used in simulating the complete autopilot system of the Piper aircraft. 
    As well as calling all libraries used in all applications throughout other programs.
    The waypoints are divided into the following fields: Position N (m), Position E (m ), Altitude (m) and Speed (m/s).

3. Run the file `executar.m` and wait until the opening of Simulink diagram `PegasusAutopilot.slx`.

4. Run the Simulink diagram `PegasusAutopilot.slx`.

## Running Simulation in Stellaris EKT-LM3S6965  

1. Open the IDE Workspace `pegasus` in folder `iar\arm\prj\pegasus` using IAR Embedded Workbench IDE.
2. Download and Debug (Ctrl + D) the code (Make sure Stellaris EKT-LM3S6965 is connected to PC using USB cable).
3. Connect ethernet cable to Stellaris EKT-LM3S6965 and set the follow configuration:

    IPv4 Address. . . . . . . . . . . : 192.168.1.107  <br />
    Subnet Mask . . . . . . . . . . . : 255.255.255.0  <br />
    Default Gateway . . . . . . . . . : 192.168.1.1  <br />
   
4. Open the folder `\matlab` in MATLAB.
5. Open the file `executar.m` and wait until the opening of Simulink diagram `PegasusAutopilot.slx`, then close it.
6. Run the Simulink diagram `bPegasusAutopilot.slx` in folder `matlab\sim\ucp\emb`.

    If any file is missing it could be copied from `matlab\lib`.

8. To see the results run the file `bizu.m` .
