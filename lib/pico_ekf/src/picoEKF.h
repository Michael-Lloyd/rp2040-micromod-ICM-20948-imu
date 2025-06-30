#ifndef PICO_EKF_H
#define PICO_EKF_H

// Include all the necessary headers
#include "picoEKFTypes.h"
#include "ExtendedKalmanFilter.h"
#include "StandardEKF.h"
#include "IterativeEKF.h"

namespace picoEKF {

// Factory function to create EKF instances
ExtendedKalmanFilter* createEKF(EKFType type = EKFType::Standard);

}

#endif