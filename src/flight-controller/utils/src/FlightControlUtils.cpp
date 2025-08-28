#include "FlightControlUtils.h"

void addVectorOfStringToProperty(yarp::os::Property& prop,
                                 std::string key,
                                 const std::vector<std::string>& list)
{
    prop.addGroup(key);

    yarp::os::Bottle& bot = prop.findGroup(key).addList();

    for (size_t i = 0; i < list.size(); i++)
    {
        bot.addString(list[i].c_str());
    }
    return;
}

yarp::os::Property paramsAsProperty(yarp::robotinterface::ParamList& params)
{
    yarp::robotinterface::ParamList p = yarp::robotinterface::mergeDuplicateGroups(params);

    yarp::os::Property prop;

    for (yarp::robotinterface::ParamList::const_iterator it = p.begin(); it != p.end(); ++it)
    {
        const yarp::robotinterface::Param& param = *it;

        // check if parentheses are balanced
        std::string stringFormatValue = param.value();
        int counter = 0;
        for (size_t i = 0; i < stringFormatValue.size() && counter >= 0; i++)
        {
            if (stringFormatValue[i] == '(')
            {
                counter++;
            } else if (stringFormatValue[i] == ')')
            {
                counter--;
            }
        }
        if (counter != 0)
        {
            yWarning() << "Parentheses not balanced for param " << param.name();
        }

        std::string s = "(" + param.name() + " " + param.value() + ")";
        prop.fromString(s, false);
    }
    return prop;
}

bool readXMLFile(
    const std::string& pathToFile,
    std::shared_ptr<BipedalLocomotion::ParametersHandler::YarpImplementation> parameterHandler)
{
    yarp::robotinterface::XMLReader reader;
    yarp::robotinterface::XMLReaderResult result = reader.getRobotFromFile(pathToFile);
    if (!result.parsingIsSuccessful)
    {
        yError() << "Could not open file " << pathToFile;
        return false;
    }
    yarp::robotinterface::ParamList params
        = result.robot.device("flight_control_cpp_config").params();
    yInfo() << "Printing the values of the XML file";
    for (auto param : params)
    {
        std::string name = param.name();
        std::string value = param.value();
        yInfo() << "Name: " << name << " Value: " << value;
    }
    yInfo() << "Read the XML file correctly!!";
    parameterHandler->set(paramsAsProperty(params));
    return true;
}

Eigen::Matrix3d fromVecToSkew(const Eigen::Vector3d& inputVect)
{
    Eigen::Matrix3d skew(3, 3);

    skew << 0, -inputVect(2), inputVect(1), inputVect(2), 0, -inputVect(0), -inputVect(1),
        inputVect(0), 0;

    return skew;
}

Eigen::Vector3d fromSkewToVec(const Eigen::Matrix3d& skewMatrix)
{
    // Given a 3x3 skew matrix, extracts its elements in a 3x1 vector
    Eigen::Vector3d outputVector;

    outputVector(0) = -skewMatrix(1, 2);
    outputVector(1) = skewMatrix(0, 2);
    outputVector(2) = -skewMatrix(0, 1);

    return outputVector;
}

bool getParameterAndCheckSize(
    const std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>& parametersHandler,
    const std::string& paramName,
    Eigen::Ref<Eigen::VectorXd> param,
    double size)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter(paramName, param))
    {
        yError() << "Parameter '" << paramName << "' not found in the config file.";
        return false;
    }
    if (param.size() != size)
    {
        yError() << "The size of the vector is not correct.";
        return false;
    }
    return true;
}
