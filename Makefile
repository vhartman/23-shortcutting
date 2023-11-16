BASE = ../rai-fork
BASE2 = ../src

OBJS = main.o

GL = 1
OMPL = 1
EIGEN = 1

DEPEND = Algo Control KOMO Core Geo Kin Gui Optim LGP Logic Manip Control PlanningSubroutines

CFLAGS += -fno-omit-frame-pointer

LIBS += -lompl -lstdc++fs

include $(BASE)/build/generic.mk
