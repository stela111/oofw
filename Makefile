.PHONY : test doc

run_test: test
	test/unittest	

test: lib
	$(MAKE) -C test

lib:
	$(MAKE) -C src

doc:
	doxygen Doxyfile

clean:
	$(MAKE) -C test clean
	$(MAKE) -C src clean
