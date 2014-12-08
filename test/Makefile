CXX=g++
RM=rm -f
CPPFLAGS=-pthread -g -std=c++11 -Wall -I.. -L../src
LDLIBS=-loofw -lgtest -lgtest_main $(CPPFLAGS)
SRCS=test_planner.cpp \
     test_stepper.cpp \
     test_delta_gantry.cpp \
     test_trapezoid.cpp \
     test_bresenham.cpp \
     test_trapezoid_generator.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

unittest : $(OBJS) ../src/liboofw.a
	$(CXX) $(LDFLAGS) -o unittest $(OBJS) $(LDLIBS)

doc : 
	doxygen Doxyfile

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	$(CXX) $(CPPFLAGS) -MM -MF .depend $^

clean:
	$(RM) $(OBJS)


include .depend
