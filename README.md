# Readme  --  Multi-Phase Simple Walker

This project shows how to compute a two-phase bipedal gait using trajectory optimization

## Entry-Point:
* GAIT\_Run/MAIN\_Run.m
  - Construct a two-phase running gait (flight - stance - flight - stance ...)
* GAIT\_Walk/MAIN\_Walk.m
  - Construct a two-phase walking gait (double stance - single stance - double stance - single stance ...)
* Derive\_Equations\_of\_Motion/MAIN\_*.m
  - Scripts that use the Matlab Symbolic toolbox to derive the dynamics for each phase of motion.

## Dependencies:
* [GPOPS-II](http://www.gpops2.com/) trajectory optimization software for Matlab.  

