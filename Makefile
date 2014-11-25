CXX=g++
RM=rm -f
CPPFLAGS=-pthread -Wall
LDLIBS=-lgtest -lgtest_main $(CPPFLAGS)
SRCS=test.cpp planner.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

unittest : $(OBJS)
	$(CXX) $(LDFLAGS) -o unittest $(OBJS) $(LDLIBS)

depend: .depend

.depend: $(SRCS)
	rm -f ./.depend
	$(CXX) $(CPPFLAGS) -MM $^>>./.depend;

clean:
	$(RM) $(OBJS)


include .depend
