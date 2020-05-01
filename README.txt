Uplod of finished project for portfolio completeness
Distributing Direct N-Body Simulations

Authors:
    Steven Johnson
    Daniel Sabsay

This directory contains three algorithms: a single-threaded, sequential implementation,
and two distributed implementations.

Running the sequential implementation:
    Works on unix1,unix2,unix3,unix4 machines in CSL.
    Compile: "make seq_sim"
    Run: "./seq_sim particles_100.in"
    Note: If using a different number of particles, "N" must be set in "sim.h".

Running the Distributed v1 algorithm:
    Works on unix1,unix2,unix3,unix4 machines in CSL.
    Compile: "make dist_sim"
    Run: "mpiexec -n 4 dist_sim particles_100.in"
    Note: If using a different number of particles, "N" must be set in "sim.h".

Running the Distributed v2 algorithm:
    WILL NOT run on CSL machines (requires MPI_THREAD_MULTIPLE support).
    Compile: "make dist2_sim"
    Run: "mpiexec -n 4 dist2_sim particles_100.in"
    Note: If using a different number of particles "N" must be set in "sim.h".
          Also, if using a different number of nodes, "NUM_NODES" must be set
          in "dist2_driver.c".
