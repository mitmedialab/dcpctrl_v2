#dcpctrl_v2
-----------------

dcpctrl_v2 contains software developed for interfacing with & controlling the Digital Construction Platform (DCP). It is the second iteration of software for the DCP. The first iteration (contained in the mdcp repo) relied on the LabJack T7 for interfacing with the AT40GW and KUKA, and ran in standard MATLAB scripts and functions. This iteration is based more heavily in Simulink, and takes advantage of Simulink's real-time capabilities (particularly Simulink Desktop Real-Time) to implement hard(er) real-time controllers.

This software is being developed during academic year 2016-2017 by the Mediated Matter Group at the MIT Media Lab.

STYLE GUIDE: https://sites.google.com/site/matlabstyleguidelines/
(Vaguely following this style guide)

STRUCTURE:
mdcp/
  *data/ -- log files, cad files, etc...SHOULD NOT COMMIT TO REPO (or, be very very very picky about what data files to commit...otherwise pulls take forever for no good reason)
    logs/
    cad/
    gcode/

  apps/ -- very high-level behaviours to run truck/arm/both
    kuka/
    at40gw/ -- examples:
      record.m
      physplayback.m
      virtplayback.m
      lightpainting.m
      foamprint.m
      home.m

  tools/ -- External tools (e.g. homeprint), perhaps move this to its own repo?
    homeprint/

  lib/ -- External dependencies (e.g. rvctools)
    rvctools/

  robots/ -- robot-specific behaviours and utility functions
    kuka/
      controllers/
      config/ -- settings and definitions
      lib/
    at40gw/
      controllers/
      config/ -- settings and definitions
      lib/
        getPos
	volt2linpos
	linpos2volt
	encoder2deg
	encoder2rad
	deg2encoder
	rad2encoder
