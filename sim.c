#include <stdio.h>
#include <stdlib.h>
#include "vector.h"
#include "sim.h"


/* Reads n lines from the given file, storing each particle's properties into the
 * given arrays. File format:
 *      <mass> (posx, posy, posz) (velx, vely, velz)
 */
void read_particles(Vector3 *positions, Vector3 *velocities, double *masses,
        int n, char *filename) {
    FILE *f;

    if ((f = fopen(filename, "r")) == NULL) {
        fprintf(stderr, "Error while opening particles file.\n");
        perror("read_particles");
        exit(1);
    }

    for (int i = 0; i < n; i++) {
        if (fscanf(f, "%lf (%lf, %lf, %lf) (%lf, %lf, %lf)\n",
                   masses + i, &(positions[i].x), &(positions[i].y), &(positions[i].z),
                   &(velocities[i].x), &(velocities[i].y), &(velocities[i].z))
                != 7) {
        fprintf(stderr, "Error while reading particles file.\n");
            perror("read_particles");
            exit(1);
        }
    }

    if (fclose(f) != 0) {
        fprintf(stderr, "Error while closing particles file.\n");
        perror("read_particles");
        exit(1);
    }
}

/* Compute the acceleration on the given particle.
 *
 * positions - array of N Vector3 structs
 * masses - array of masses (length n)
 * p - particle number
 * n - number of bodies
 */
Vector3 get_acc(Vector3 *positions, double *masses, int p, int n) {
    Vector3 acc = {0, 0, 0};

    for (int o = 0; o < n; o++) {
        if (o == p) {
            continue;
        }
        Vector3 pVector = positions[p];
        Vector3 oVector = positions[o];
        Vector3 pToO = vector_from_to(pVector, oVector);
        double f = (G * masses[p] * masses[o]) / VECTOR_LEN_SQRD(pToO);
        Vector3 force_dir = normalize_vector(pToO);
        Vector3 force = scale_vector(force_dir, f);
        ADD_TO_VECTOR(acc, force);
    }

    acc = scale_vector(acc, 1 / masses[p]);

    return acc;
}

/* Compute timestep.
 *
 * prevs - array of [posVector, velVector] at previous timestep
 *         (length 2n)
 * masses - array of masses (length n)
 * n - number of particles
 * t - current timestep number
 * delta_t - timestep interval (in seconds)
 * first - index of first particle to update
 * last - index of last particle to update
 */
/* void do_timestep(Vector3 *prevs, double *masses, int n, int t, double delta_t, */
/*         int first, int last) { */
/*     for (int p = first; p <= last; p++) { */
/*         /1* Leapfrog integration *1/ */
/*         /1* Update position *1/ */
/*         Vector3 pos = prevs[p*2];  /1* Get previous position *1/ */
/*         Vector3 vel = prevs[(p*2)+1];  /1* Get previous velocity *1/ */
/*         vel = scale_vector(vel, delta_t); */
/*         ADD_TO_VECTOR(pos, vel); */
/*         prevs[p*2] = pos; */
/*     } */

/*     for (int p = first; p <= last; p++) { */
/*         /1* Update velocity *1/ */
/*         Vector3 acc = get_acc(prevs, masses, p, n); */
/*         acc = scale_vector(acc, delta_t); */
/*         ADD_TO_VECTOR(prevs[(p*2)+1], acc); */
/*     } */

/* } */

/* Computes the new positions for the timestep, t.
 *
 * positions - array of N Vector3 structs
 * velocities - Array of N Vector3 structs in the case of sequential algorithm.
 *              Array of N/num_nodes Vector3 structs for the distributed
 *                  algorithm
 * masses - array of masses (length n)
 * n - number of particles
 * t - current timestep number
 * delta_t - timestep interval (in seconds)
 * first - index of first particle to update
 * last - index of last particle to update
 */
void compute_positions(Vector3 *positions, Vector3 *velocities, double *masses,
        int n, int t, double delta_t, int first, int last) {
    for (int p = first; p <= last; p++) {
        /* Leapfrog integration */
        /* Update position */
        Vector3 pos = positions[p];  /* Get previous position */
        Vector3 vel = velocities[p];  /* Get previous velocity */
        vel = scale_vector(vel, delta_t);
        ADD_TO_VECTOR(pos, vel);
        positions[p] = pos;
    }
}



/* Computes the new velocities for the timestep, t, which is actualy 1/2 of
 * a timestep ahead of t.
 *
 * positions - array of N Vector3 structs
 * velocities - Array of N Vector3 structs in the case of sequential algorithm.
 *              Array of N/num_nodes Vector3 structs for the distributed
 *                  algorithm
 * masses - array of masses (length n)
 * n - number of particles
 * t - current timestep number
 * delta_t - timestep interval (in seconds)
 * first - index of first particle to update
 * last - index of last particle to update
 */
void compute_velocities(Vector3 *positions, Vector3 *velocities, double *masses,
        int n, int t, double delta_t, int first, int last) {
    for (int p = first; p <= last; p++) {
        /* Update velocity */
        Vector3 acc = get_acc(positions, masses, p, n);
        acc = scale_vector(acc, delta_t);
        ADD_TO_VECTOR(velocities[p], acc);
    }
}
