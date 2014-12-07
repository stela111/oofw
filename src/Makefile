CXX=g++
AR=ar
RM=rm -f
CPPFLAGS=-pthread -g -std=c++11 -Wall
LDLIBS= $(CPPFLAGS)
SRCS=planner.cpp stepper.cpp delta_gantry.cpp trapezoid_ticker.cpp trapezoid_generator.cpp
OBJS=$(subst .cpp,.o,$(SRCS))
LIB=liboofw.a

$(LIB) : $(OBJS)
	$(AR) rcs $(LIB) $(OBJS)

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	$(CXX) $(CPPFLAGS) -MM -MF .depend $^

clean:
	$(RM) $(OBJS) $(LIB)


include .depend
