#include <math.h>
#include "vector.h"

Vector3 normalize_vector(Vector3 v) {
    Vector3 norm;
    double mag = sqrt(VECTOR_LEN_SQRD(v));

    norm.x = v.x / mag;
    norm.y = v.y / mag;
    norm.z = v.z / mag;

    return norm;
}

Vector3 scale_vector(Vector3 v, double scalar) {
    Vector3 scaled;
    scaled.x = scalar * v.x;
    scaled.y = scalar * v.y;
    scaled.z = scalar * v.z;

    return scaled;
}

Vector3 vector_from_to(Vector3 from, Vector3 to) {
    Vector3 from_to;
    from_to.x = to.x - from.x;
    from_to.y = to.y - from.y;
    from_to.z = to.z - from.z;

    return from_to;
}
