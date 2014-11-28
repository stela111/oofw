.PHONY : test

run_test: test
	test/unittest	

test:
	$(MAKE) -C test

doc:
	doxygen Doxyfile

clean:
	$(MAKE) -C test clean
