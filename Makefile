.PHONY : test

run_test: test lib
	test/unittest	

test:
	$(MAKE) -C test

lib:
	$(MAKE) -C src

doc:
	doxygen Doxyfile

clean:
	$(MAKE) -C test clean
	$(MAKE) -C src clean
