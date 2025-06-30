#include "picoEKF.h"

namespace picoEKF {

ExtendedKalmanFilter* createEKF(EKFType type) {
    switch (type) {
        case EKFType::Standard:
            return new StandardEKF();
        case EKFType::Iterative:
            return new IterativeEKF();
        // Future EKF types can be added here
        default:
            return new StandardEKF();
    }
}

}