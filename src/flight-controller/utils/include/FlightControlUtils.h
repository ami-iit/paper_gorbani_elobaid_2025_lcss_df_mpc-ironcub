#ifndef FLIGHT_CONTROL_UTILS_H
#define FLIGHT_CONTROL_UTILS_H

/**
 * @brief Utility functions used by the flight controller.
 *
 * Provides utility functions to be used in the flight controller.
 */

// YARP headers
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/robotinterface/Types.h>
#include <yarp/robotinterface/XMLReader.h>
#include <yarp/sig/Vector.h>

// iDynTree headers
#include <Eigen/Dense>
#include <iDynTree/EigenHelpers.h>

// BLF headers
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

void addVectorOfStringToProperty(yarp::os::Property& prop,
                                 std::string key,
                                 const std::vector<std::string>& list);

yarp::os::Property paramsAsProperty(yarp::robotinterface::ParamList& params);

bool readXMLFile(
    const std::string& pathToFile,
    std::shared_ptr<BipedalLocomotion::ParametersHandler::YarpImplementation> parameterHandler);

Eigen::Matrix3d fromVecToSkew(const Eigen::Vector3d& inputVect);

Eigen::Vector3d fromSkewToVec(const Eigen::Matrix3d& skewMatrix);

bool getParameterAndCheckSize(
    const std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>& parametersHandler,
    const std::string& paramName,
    Eigen::Ref<Eigen::VectorXd> param,
    double size = 3);

#endif /* end of include guard FLIGHT_CONTROL_UTILS_H */
