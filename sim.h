#ifndef NBODYSIM_H
#define NBODYSIM_H

#include "vector.h"

typedef struct body {
    Vector3* positions;
    Vector3* velocities;
    double mass;
} Body;

enum chunk_status {NOT_RECEIVED, RECEIVED, COMPUTED};

#define G 6.67408e-11  /* gravitational constant */

#define N 100

Vector3 get_acc(Vector3 *prevs, double *masses, int p, int n);
/* void do_timestep(Vector3 *prevs, double *masses, int n, int t, double delta_t, */
/*         int first, int last); */
void compute_positions(Vector3 *positions, Vector3 *velocities, double *masses,
        int n, int t, double delta_t, int first, int last);
void compute_velocities(Vector3 *positions, Vector3 *velocities, double *masses,
        int n, int t, double delta_t, int first, int last);
void dist2_compute_velocities(Vector3 *positions, Vector3 *velocities,
        double *masses, int n, double delta_t, int first, int last, int rank);
void read_particles(Vector3 *positions, Vector3 *velocities, double *masses,
        int n, char *filename);

#endif
