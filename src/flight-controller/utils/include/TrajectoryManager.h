#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <iDynTree/EigenHelpers.h>
#include <matio.h>
#include <string>
#include <unordered_map>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

class Trajectory
{
public:
    /**
     * @brief Construct a new Trajectory object
     *
     */
    Trajectory();
    /**
     * @brief Construct a new Trajectory object
     *
     * @param name name of the trajectory, coming from the .mat file
     * @param values vector containing the numerical values of the trajectory
     * @param fps frames-per-second
     */
    Trajectory(const std::string& name, const std::vector<Eigen::VectorXd>& values, const int fps);
    /**
     * @brief Destroy the Trajectory object
     *
     */
    ~Trajectory() = default;
    /**
     * @brief Get the trajectory value at idx
     *
     * @param idx the index that refers to the istant in the trajectory
     * @return Eigen::Ref<const Eigen::VectorXd> the vector of the trajectory at instant idx
     */
    Eigen::Ref<const Eigen::VectorXd> getValue(int idx) const;
    /**
     * @brief Resample the trajectory if the desired fps is higher than the one in the data
     *
     * @param des_fps the desired frames-per-second
     * @return true
     * @return false
     */
    bool upsample(int des_fps);
    /**
     * @brief Get the length of the trajectory
     *
     * @return int the length of the trajectory
     */
    int getLength() const;

private:
    std::string name;
    std::vector<Eigen::VectorXd> values;
    int fps;
    int length;
};

class TrajectoryManager
{
public:
    /**
     * @brief Construct a new Trajectory Manager reading from the .ini file
     *
     * @param parametersHandler the parameters handler
     * @param des_fps the desired frames-per-second
     * @return true
     * @return false
     */
    bool configure(
        std::shared_ptr<BipedalLocomotion::ParametersHandler::YarpImplementation> parametersHandler,
        int des_fps);
    /**
     * @brief load a trajectory from a file
     *
     * @param filename path to the file containing the trajectory
     * @param trajectoryName name of the trajectory to be loaded
     * @param des_fps the desired frames-per-second
     * @return true
     * @return false
     */
    bool loadTrajectoryFromFile(const std::string& trajectoryName, int des_fps);
    /**
     * @brief Advance the trajectory by one step
     *
     * @return true if the trajectory has been advanced
     */
    bool advanceTrajectory();
    /**
     * @brief Get the current trajectory index
     *
     * @return int
     */
    int getTrajectoryIndex() const;
    /**
     * @brief Get the current value of a trajectory
     *
     * @param key name of the trajectory
     * @return Eigen::VectorXd
     */
    Eigen::VectorXd getCurrentValue(std::string key) const;

private:
    // Trajectory trajectory;
    mat_t* mat;
    int trajectoryIndex = 0;
    int trajectorySize;
    std::unordered_map<std::string, std::shared_ptr<Trajectory>> trajectories_map;
};

#endif // TRAJECTORY_MANAGER_H
