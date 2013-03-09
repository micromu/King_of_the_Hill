
.PHONY:

all:
	@$(MAKE) -C ../../ARDroneLib/Soft/Build USE_LINUX=yes
	@$(MAKE) -C video_demo/Build USE_LINUX=yes

$(MAKECMDGOALS):
	@$(MAKE) -C ../../ARDroneLib/Soft/Build USE_LINUX=yes $(MAKECMDGOALS)
	@$(MAKE) -C video_demo/Build USE_LINUX=yes $(MAKECMDGOALS)
