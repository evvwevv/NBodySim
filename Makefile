CC = gcc
CFLAGS = -x c -g -Wall -Werror -std=c99

dist2_sim: dist2_driver.c sim.c sim.h vector.c vector.h
	mpicc $(CFLAGS) -o dist2_sim dist2_driver.c sim.c sim.h vector.c vector.h -lm

dist_sim: dist_driver.c sim.c sim.h vector.c vector.h
	mpicc $(CFLAGS) -o dist_sim dist_driver.c sim.c sim.h vector.c vector.h -lm

seq_sim: seq_driver.c sim.c sim.h vector.c vector.h
	$(CC) $(CFLAGS) -o seq_sim seq_driver.c sim.c sim.h vector.c vector.h -lm

test_runner: test_runner.c test_main.c culite.h vector.h vector.c sim.h sim.c
	$(CC) $(CFLAGS) -o test_runner test_runner.c test_main.c culite.h vector.h vector.c sim.c sim.h

.PHONY: test
test:
	$(MAKE) test_runner
	@echo ""
	./test_runner
