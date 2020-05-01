/* seq_driver.c
 *
 * This is a driver for the sequential version of the simulation.
 */
#include <stdio.h>
#include <stdlib.h>
#include "vector.h"
#include "sim.h"

Vector3 positions[N];
Vector3 velocities[N];
double masses[N];

void output_frame(Vector3 *positions, int n) {
    for (int i = 0; i < n; i++) {
        Vector3 pos = positions[i];
        printf("Particle %d: (%f, %f, %f)\n", i, pos.x, pos.y, pos.z); 
    }
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        fprintf(stderr, "Input filename expected.\n");
        exit(1);
    }

    // Initialize
    /* positions[0].x = 0; */
    /* positions[0].y = 0; */
    /* positions[0].z = 0; */
    /* velocities[0].x = 0; */
    /* velocities[0].y = 0; */
    /* velocities[0].z = 0; */

    /* positions[1].x = 1000000; */
    /* positions[1].y = 0; */
    /* positions[1].z = 0; */
    /* velocities[1].x = 0; */
    /* velocities[1].y = 50; */
    /* velocities[1].z = 0; */

    /* positions[2].x = 1000000000; */
    /* positions[2].y = 0; */
    /* positions[2].z = 0; */
    /* velocities[2].x = 0; */
    /* velocities[2].y = 20; */
    /* velocities[2].z = 0; */

    /* masses[0] = 2e20; */
    /* masses[1] = 2e5; */
    /* masses[2] = 3e2; */

    read_particles(positions, velocities, masses, N, argv[1]);

    int timesteps = 20;
    double delta_t = 1.0;

    for (int i = 0; i < timesteps; i++) {
        /* do_timestep(prevs, masses, N, i, delta_t, 0, N - 1); */
        compute_positions(positions, velocities, masses, N, i, delta_t, 0, N - 1);
        compute_velocities(positions, velocities, masses, N, i, delta_t, 0, N - 1);
        /* new position is now in prevs */
        output_frame(positions, N);
    }

};
