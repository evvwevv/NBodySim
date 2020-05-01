/* dist_2_driver.c
 *
 * This is a driver for the non-blocking version of the distributed algorithm.
 */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include "vector.h"
#include "sim.h"

#include "mpi.h"

#define NUM_NODES 4

Vector3 positions[N];
Vector3 velocities[N];
Vector3 sum_forces[N];
double masses[N];
int rank;
int particles_per_node;

pthread_mutex_t chunk_status_lock;

void output_frame(Vector3 *positions, int n) {
    for (int i = 0; i < n; i++) {
        Vector3 pos = positions[i];
        printf("Particle %d: (%f, %f, %f)\n", i, pos.x, pos.y, pos.z); 
    }
}

int main(int argc, char *argv[]) {
    int num_nodes;
    int root = 0;

    if (argc != 2) {
        fprintf(stderr, "Expected input filename.\n");
        exit(1);
    }

    /* MPI_Init(&argc, &argv); */
    int provided_thread_support;
    MPI_Init_thread(&argc, &argv, MPI_THREAD_MULTIPLE, &provided_thread_support);
    if (provided_thread_support != MPI_THREAD_MULTIPLE) {
        fprintf(stderr, "This program requires a version of MPI with "
                        "MPI_THREAD_MULTIPLE support.\n");
        exit(1);
    }
    MPI_Comm_size(MPI_COMM_WORLD, &num_nodes);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);

    /* if (rank == root && N % num_nodes != 0) { */
    if (rank == root && num_nodes != NUM_NODES) {
        fprintf(stderr, "Number of nodes must be same as NUM_NODES defined in "
                        "dist2_driver.c\n");
        exit(1);
    }
    if (rank == root && N % num_nodes != 0) {
        fprintf(stderr, "Number of nodes must be a factor of N.\n");
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

    /* Divide particles into chunks */
    particles_per_node = N / num_nodes;
    int first = rank * particles_per_node;
    int last = first + particles_per_node - 1;

    for (int i = 0; i < timesteps; i++) {
        /* Compute new positions */
        compute_positions(positions, velocities, masses, N, i, delta_t, first, last);
        /* Compute new velocities, exchanging positions in separate threads */
        dist2_compute_velocities(positions, velocities, masses, N, delta_t,
                first, last, rank);

        if (rank == root) {
            output_frame(positions, N);
        }

        /* printf("rank %d: timestep %d complete\n", rank, i); */
    }
    MPI_Finalize();
};

/* Returns 1 if all chunks have been received. */
int all_received(enum chunk_status *chunks, int n) {
    /* TODO: Do we need to lock the chunks array here? */
    for (int i = 0; i < n; i++) {
        if (chunks[i] < RECEIVED) {
            return 0;
        }
    }

    return 1;
}

/* Send the node's chunk of particle positions to all other nodes. */
void *dist2_send(void *arg) {
    /* MPI_Request reqs[NUM_NODES]; */

    for (int i = 0; i < NUM_NODES; i++) {
        if (i == rank) {
            continue;
        }
        /* MPI_Isend(positions + (rank * particles_per_node), */
        /*           particles_per_node * sizeof(Vector3), */
        /*           MPI_DOUBLE, */
        /*           i, */
        /*           0,  /1* tag *1/ */
        /*           MPI_COMM_WORLD, */
        /*           &(reqs[i])); */
        /* MPI_Request_free(reqs + i); */
        MPI_Send(positions + (rank * particles_per_node),
                 particles_per_node * sizeof(Vector3),
                 MPI_UNSIGNED_CHAR,
                 i,
                 0,  /* tag */
                 MPI_COMM_WORLD);
    }

    /* printf("rank %d: all data has been sent\n", rank); */
    return NULL;
}

void print_chunk_status(enum chunk_status *chunks, int n) {
    printf("chunk status:\n");
    for (int i = 0; i < n; i++) {
        printf("  %d: %d\n", i, chunks[i]);
    }
}

/* The routine for the gather thread.
 *
 * chunks - pointer to array of chunk_status flags used by the compute thread.
 *          NOTE: Access to this must be synchronized!
 */
void *dist2_gather(void *chunksp) {
    enum chunk_status *chunks = chunksp;
    MPI_Request reqs[NUM_NODES];
    char done[NUM_NODES] = {0};

    /* Start non-blocking receives */
    for (int i = 0; i < NUM_NODES; i++) {
        if (i == rank) {
            continue;
        }
        MPI_Irecv(positions + (i * particles_per_node),
                  particles_per_node * sizeof(Vector3),
                  MPI_UNSIGNED_CHAR,
                  i,
                  0,  /* Tag */
                  MPI_COMM_WORLD,
                  &(reqs[i]));
    }

    /* printf("rank: %d: receives started\n", rank); */

    /* Loop until all nodes' data has been received */
    int flag;
    while (all_received(chunks, NUM_NODES) == 0) {
        for (int i = 0; i < NUM_NODES; i++) {
            if (i == rank) {
                continue;
            }
            /* printf("rank: %d checking request %d\n", rank, i); */
            if (done[i]) {
                continue;
            }
            MPI_Test(&(reqs[i]), &flag, MPI_STATUS_IGNORE);
            if (flag != 0) {  /* TODO: true? */
                /* lock chunks */
                if (pthread_mutex_lock(&chunk_status_lock) != 0) {
                    fprintf(stderr, "Error locking mutex.\n");
                    exit(1);
                }
                /* update chunks */
                chunks[i] = RECEIVED;
                /* unlock chunks */
                if (pthread_mutex_unlock(&chunk_status_lock) != 0) {
                    fprintf(stderr, "Error unlocking mutex.\n");
                    exit(1);
                }
                done[i] = 1;
            }
        }
    }

    /* printf("rank %d: gather complete\n", rank); */

    return NULL;
}

/* Returns 1 if all chunks have been computed. 0 otherwise. */
int all_computed(enum chunk_status *chunks, int n) {
    for (int i = 0; i < n; i++) {
        if (chunks[i] != COMPUTED) {
            return 0;
        }
    }
    return 1;
}

void dist2_sum_forces_for_own_chunk(Vector3 *positions, Vector3 *sum_forces,
        double *masses, int n, int first, int last, int chunk_num) {
    if (chunk_num != rank) {
        fprintf(stderr, "Invalid use of dist2_sum_forces_for_own_chunk!\n");
        exit(1);
    }
    for (int p = first; p <= last; p++) {
        for (int o = chunk_num * particles_per_node;
                o < (chunk_num + 1) * particles_per_node; o++) {
            if (o == p) {
                continue;
            }
            Vector3 pVector = positions[p];
            Vector3 oVector = positions[o];
            Vector3 pToO = vector_from_to(pVector, oVector);
            double f = (G * masses[p] * masses[o]) / VECTOR_LEN_SQRD(pToO);
            Vector3 force_dir = normalize_vector(pToO);
            Vector3 force = scale_vector(force_dir, f);
            ADD_TO_VECTOR(sum_forces[p], force);
        }
    }
}

/* Adds the forces on rank's particles due to the particles from another chunk
 * given by chunk_num and stores the result in sum_forces.
 *
 * chunk_num - number of chunk not equal to this node's chunk (i.e. it must be
 *             a different chunk)
 */
void dist2_sum_forces_for_chunk(Vector3 *positions, Vector3 *sum_forces,
        double *masses, int n, int first, int last, int chunk_num) {
    if (chunk_num == rank) {
        fprintf(stderr, "Invalid use of of dist2_sum_forces_for_chunk!\n");
        exit(1);
    }
    for (int p = first; p <= last; p++) {
        for (int o = chunk_num * particles_per_node;
                o < (chunk_num + 1) * particles_per_node; o++) {
            Vector3 pVector = positions[p];
            Vector3 oVector = positions[o];
            Vector3 pToO = vector_from_to(pVector, oVector);
            double f = (G * masses[p] * masses[o]) / VECTOR_LEN_SQRD(pToO);
            Vector3 force_dir = normalize_vector(pToO);
            Vector3 force = scale_vector(force_dir, f);
            ADD_TO_VECTOR(sum_forces[p], force);
        }
    }
}

/* Sums all forces and stores result in sum_forces. Returns when all
 * chunks have been received and computed.
 *
 * sum_forces - array of N Vector3's where forces are accumulated
 * first - index of first particle in chunk
 * last - index of last particle in chunk
 * chunks - pointer to array of chunk_status indicating status of
 *          position information from each node
 */
void dist2_sum_forces(Vector3 *positions, Vector3 *sum_forces, double *masses,
        int n, int first, int last, enum chunk_status *chunks) {
    while (all_computed(chunks, NUM_NODES) != 1) {
        for (int i = 0; i < NUM_NODES; i++) {
            /* TODO: Do we need to lock chunks here? */
            if (chunks[i] != RECEIVED) {
                continue;
            }
            /* Sum forces for new data */
            if (i == rank) {
                dist2_sum_forces_for_own_chunk(positions, sum_forces, masses, n,
                        first, last, i);
            } else {
                dist2_sum_forces_for_chunk(positions, sum_forces, masses, n,
                        first, last, i);
            }
            /* lock chunks */
            if (pthread_mutex_lock(&chunk_status_lock) != 0) {
                fprintf(stderr, "Error locking mutex.\n");
                exit(1);
            }
            chunks[i] = COMPUTED;
            /* unlock chunks */
            if (pthread_mutex_unlock(&chunk_status_lock) != 0) {
                fprintf(stderr, "Error unlocking mutex.\n");
                exit(1);
            }
        }
    }
}

/* Updates the velocities for this node's chunk of particles. Assumes that
 * sum_forces contains the sum of forces on the chunk's particles due to
 * all other N - 1 particles (i.e. the particles from all other nodes).
 */
void dist2_update_velocities(Vector3 *velocities, Vector3 *sum_forces,
        double *masses, int first, int last, int delta_t) {
    for (int i = first; i <= last; i++) {
        Vector3 acc = scale_vector(sum_forces[i], 1 / masses[i]);
        Vector3 delta_v = scale_vector(acc, delta_t);
        ADD_TO_VECTOR(velocities[i], delta_v);
    }
}

/* Zeros the vectors in sum_forces. */
void dist2_zero_sum_forces(Vector3 *sum_forces, int first, int last) {
    /* memset(sum_forces + (first * particles_per_node), 0, */
    /*        particles_per_node * sizeof(Vector3)); */
    for (int i = first; i <= last; i++) {
        sum_forces[i].x = 0;
        sum_forces[i].y = 0;
        sum_forces[i].z = 0;
    }
}

/* Compute new velocities for the current timestep, which is actually 1/2 of
 * a timestep ahead of t.
 *
 * This function computes with available data, waiting for data from all nodes
 * to arrive.
 */
void dist2_compute_velocities(Vector3 *positions, Vector3 *velocities,
        double *masses, int n, double delta_t, int first, int last, int rank) {
    /* Array of flags, indicating when we have the data from each node */
    /* New thread */
        /* Issue receives from all nodes */
        /* When recieve data, write to positions array */
        /* Lock flag_array */
        /* Update flag_array */
        /* Unlock flag_array */
        /* When all nodes have been "read", kill thread */
    /* Compute thread */
        /* Until flag_array is all COMPUTED */
            /* Wait until a new set of data is available */
            /* Compute forces for that data and add to accumulation */
            /* lock flag_array */
            /* Update flag_array */
            /* unlock flag_array */
        /* Now accumulation has sum of forces. */
        /* Divide accumulation by m -> acc */
        /* Compute velocities */
    if (pthread_mutex_init(&chunk_status_lock, NULL) != 0) {
        fprintf(stderr, "Error initializing mutex.\n");
        exit(1);
    }
    enum chunk_status chunks[NUM_NODES] = {NOT_RECEIVED};
    chunks[rank] = RECEIVED;

    pthread_t send_thread;
    if (pthread_create(&send_thread, NULL, dist2_send, NULL) != 0) {
        fprintf(stderr, "Error occurred creating thread!\n");
        exit(1);
    }

    pthread_t gather_thread;
    if (pthread_create(&gather_thread, NULL, dist2_gather, chunks) != 0) {
        fprintf(stderr, "Error occurred creating thread!\n");
        exit(1);
    }

    /* printf("rank %d: starting computation\n", rank); */
    /* Compute thread */
    dist2_zero_sum_forces(sum_forces, first, last);
    dist2_sum_forces(positions, sum_forces, masses, N, first, last, chunks);
    /* printf("rank %d: starting update_velocities\n", rank); */
    dist2_update_velocities(velocities, sum_forces, masses, first, last, delta_t);

    if (pthread_join(send_thread, NULL) != 0) {
        fprintf(stderr, "Error joining with send thread.\n");
        exit(1);
    }
}
