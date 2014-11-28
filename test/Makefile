CXX=g++
RM=rm -f
CPPFLAGS=-pthread -std=c++11 -Wall -I../src -I../fake
LDLIBS=-lgtest -lgtest_main $(CPPFLAGS)
SRCS=test.cpp test_stepper.cpp 	../src/planner.cpp ../src/stepper.cpp
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
