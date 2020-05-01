#ifndef NBODYSIM_VECTOR_H
#define NBODYSIM_VECTOR_H

typedef struct vector3 {
    double x;
    double y;
    double z;
} Vector3;

Vector3 vector_from_to(Vector3 from, Vector3 to);
Vector3 normalize_vector(Vector3);
Vector3 scale_vector(Vector3 v, double scalar);

#define VECTOR_LEN_SQRD(v) \
    ((v.x * v.x) + (v.y * v.y) + (v.z * v.z))

#define ADD_TO_VECTOR(v1, v2) \
    v1.x += v2.x; \
    v1.y += v2.y; \
    v1.z += v2.z

#endif
