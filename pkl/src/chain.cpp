#include "pkl/chain.hpp"
#include "pkl/typedef.h"

namespace PKL {
PKL::Scalar singularity_safe(PKL::Scalar s) {
    // TODO: check for the right value of eps
    static const Scalar eps = 1E-5;
    return s < eps ? 0.0 : 1.0 / s;
}
}  // namespace PKL
