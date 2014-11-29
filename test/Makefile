CXX=g++
RM=rm -f
CPPFLAGS=-pthread -std=c++11 -Wall -I..
LDLIBS=-lgtest -lgtest_main $(CPPFLAGS)
SRCS=test.cpp test_stepper.cpp test_coordinated_axes.cpp ../src/planner.cpp ../src/stepper.cpp ../src/coordinated_axes.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

unittest : $(OBJS)
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
