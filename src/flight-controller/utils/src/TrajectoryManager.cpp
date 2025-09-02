#include "TrajectoryManager.h"

Trajectory::Trajectory(const std::string& name,
                       const std::vector<Eigen::VectorXd>& values,
                       const int fps)
    : name(name)
    , values(values)
    , fps(fps)
{
    length = values.size();
}

Eigen::Ref<const Eigen::VectorXd> Trajectory::getValue(int idx) const
{
    return values[idx];
}

int Trajectory::getLength() const
{
    return length;
}

bool Trajectory::upsample(int des_fps)
{
    std::vector<Eigen::VectorXd> resampled;
    double ratio = static_cast<double>(des_fps) / fps;
    for (size_t i = 0; i < values.size() - 1; i++)
    {
        for (size_t k = 0; k < ratio; k++)
        {
            Eigen::VectorXd interpolatedVal = values[i] + (values[i + 1] - values[i]) * (k / ratio);
            resampled.push_back(interpolatedVal);
        }
    }
    values = resampled;
    fps = des_fps;
    length = values.size();
    return true;
}

bool TrajectoryManager::configure(
    std::shared_ptr<BipedalLocomotion::ParametersHandler::YarpImplementation> parametersHandler,
    int des_fps)
{
    std::string trajectoryName;
    BipedalLocomotion::ParametersHandler::IParametersHandler::shared_ptr
        groupHandler_trajectoryManager
        = parametersHandler->getGroup("TRAJECTORY_MANAGER").lock();
    if (!groupHandler_trajectoryManager)
    {
        yError() << "Group [TRAJECTORY_MANAGER] not found in the config file.";
        return false;
    }
    if (!groupHandler_trajectoryManager->getParameter("trajectoryFile", trajectoryName))
    {
        yError() << "Unable to read the name of the trajectory.";
        return false;
    }
    if (!this->loadTrajectoryFromFile(trajectoryName, des_fps))
    {
        yError() << "Unable to load the trajectory from file.";
        return false;
    }
    return true;
}

bool TrajectoryManager::loadTrajectoryFromFile(const std::string& filename, const int des_fps)
{
    yInfo() << "Loading trajectory from file " << filename;

    yarp::os::ResourceFinder rf;
    std::string pathToMathFile = rf.findFileByName(filename);
    yInfo() << "Path to file: " << pathToMathFile;

    mat = Mat_Open(pathToMathFile.c_str(), MAT_ACC_RDONLY);
    if (mat == nullptr)
    {
        yError() << "Error opening file " << filename;
        return false;
    }

    // read the keys of the file
    matvar_t* matvar = nullptr;

    // read from mat file the fps
    matvar_t* fps_mat = Mat_VarRead(mat, "fps");
    if (fps_mat == nullptr)
    {
        yError() << "Error reading fps";
        return false;
    }
    int fps = static_cast<int>(*static_cast<double*>(fps_mat->data));

    trajectorySize = 0;

    for (matvar = Mat_VarReadNextInfo(mat); matvar != nullptr; matvar = Mat_VarReadNextInfo(mat))
    {
        yInfo() << "Reading Key:" << matvar->name;

        // access and store the data
        matvar_t* matVar = Mat_VarRead(mat, matvar->name);
        if (matVar == nullptr)
        {
            yError() << "Error reading " << matvar->name << " variable";
            return false;
        }
        std::vector<Eigen::VectorXd> trajectory_vec;

        Eigen::Map<Eigen::MatrixXd> trajectory_eigen_map(static_cast<double*>(matVar->data),
                                                         matVar->dims[0],
                                                         matVar->dims[1]);
        for (int i = 0; i < trajectory_eigen_map.cols(); ++i)
        {
            trajectory_vec.push_back(trajectory_eigen_map.col(i));
        }
        std::shared_ptr<Trajectory> trajectoryPtr
            = std::make_shared<Trajectory>(matvar->name, trajectory_vec, fps);

        trajectories_map[matvar->name] = trajectoryPtr;

        if (fps != des_fps && matVar->dims[1] > 1)
        {
            yInfo() << "The desired fps is" << des_fps << "while the fps of the trajectory is"
                    << fps << ". Resampling" << matvar->name << "trajectory.";
            trajectories_map[matvar->name]->upsample(des_fps);
        }

        if (trajectories_map[matvar->name]->getLength() > trajectorySize)
        {
            trajectorySize = trajectories_map[matvar->name]->getLength();
        }
    }

    yInfo() << "Trajectory has" << trajectorySize << "samples";
    yInfo() << "Fps:" << fps << "- time-interval: " << 1.0 / fps;

    // TODO: find a cleaner way to get the size of the trajectory

    return true;
}

bool TrajectoryManager::advanceTrajectory()
{
    if (trajectoryIndex < trajectorySize - 1)
    {
        trajectoryIndex++;
    } else
    {
        // TODO: check if this is the desired behavior
        // yInfo() << "Trajectory finished. Staying at last point";
    }
    return true;
}

int TrajectoryManager::getTrajectoryIndex() const
{
    return trajectoryIndex;
}

Eigen::VectorXd TrajectoryManager::getCurrentValue(std::string key) const
{
    if (trajectories_map.find(key) == trajectories_map.end())
    {
        yWarning() << "Key " << key << " not found.";
    }
    return trajectories_map.at(key)->getValue(trajectoryIndex);
}

/* Example
 *
 * during the update method of the flight controller, the trajectory is advanced
 * trajectoryManager.advanceTrajectory();
 * then the thrust, joint position (and whatever quantity is stored) are retrieved
 * Eigen::VectorXd thrustRef = trajectoryManager.getCurrentValue("thrust");
 * Eigen::VectorXd jointPositionsRef = trajectoryManager.getCurrentValue("joint_positions");
 *
 */
