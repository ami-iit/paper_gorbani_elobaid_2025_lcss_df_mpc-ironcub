#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <configFolder/ConfigFolderPath.h>
#include <fstream>
#include <random>
#include <simpleExample/SimpleExampleMPC.h>
#include <vector>

int main()
{
    SimpleExampleMPC mpc;
    auto parameterHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    parameterHandler->setFromFile(getConfigPath() + "/simple-mpc-config.ini");

    // ---------- Subsystem α ----------
    Eigen::Matrix2d A_alpha;
    A_alpha << 0, 1, -2, -3;
    Eigen::Vector2d B_alpha;
    B_alpha << 0, 1;
    Eigen::RowVector2d C_alpha;
    C_alpha << 1, 0;

    // ---------- Subsystem β ----------
    Eigen::Matrix2d A_beta;
    A_beta << 0, 1, -5, -6;
    Eigen::Vector2d B_beta;
    B_beta << 0, 1;
    Eigen::RowVector2d C_beta;
    C_beta << 1, 0;

    // Coupling matrix E (2x1)
    Eigen::Vector2d E;
    E << 1, 0;

    // ---------- Composite system ----------
    Eigen::Matrix4d A_c = Eigen::Matrix4d::Zero();
    A_c.block<2, 2>(0, 0) = A_alpha;
    A_c.block<2, 2>(2, 2) = A_beta;
    A_c.block<2, 2>(0, 2) = E * C_beta; // fixed: 2x2 coupling

    Eigen::Matrix<double, 4, 2> B_c = Eigen::Matrix<double, 4, 2>::Zero();
    B_c.block<2, 1>(0, 0) = B_alpha;
    B_c.block<2, 1>(2, 1) = B_beta;

    Eigen::Matrix<double, 2, 4> C_c = Eigen::Matrix<double, 2, 4>::Zero();
    C_c.block<1, 2>(0, 0) = C_alpha;
    C_c.block<1, 2>(1, 2) = C_beta;

    // ---------- Simulation parameters ----------
    // compute the discrete time for the subssystem alpha and beta
    double dt; // time step
    parameterHandler->getParameter("periodMPC", dt);
    Eigen::Matrix2d A_alpha_discrete = A_alpha * dt + Eigen::Matrix2d::Identity();
    Eigen::Matrix2d A_beta_discrete = A_beta * dt + Eigen::Matrix2d::Identity();
    Eigen::Vector2d B_alpha_discrete = B_alpha * dt;
    Eigen::Vector2d B_beta_discrete = B_beta * dt;
    Eigen::Vector2d E_discrete = E * dt;
    Eigen::Vector4d x = Eigen::Vector4d::Zero(); // initial state
    Eigen::Vector2d u = Eigen::Vector2d::Zero(); // initial input
    unsigned int seed = 42; // Use a fixed seed for reproducibility
    std::mt19937 gen(seed);
    std::bernoulli_distribution dist(0.5);
    double noise_bound = 0.001;
    std::uniform_real_distribution<double> noise_dist(-noise_bound, noise_bound);
    Eigen::Vector2d x_beta = Eigen::Vector2d::Zero();

    // compute the observability matrix of subsystem beta
    Eigen::Matrix2d O_beta;
    O_beta << C_beta, C_beta * A_beta;

    int N = 300;
    std::vector<double> inputData(N);
    std::vector<double> outputData(N);

    for (int i = 0; i < N; ++i)
    {
        inputData[i] = dist(gen) ? 3.0 : 0.0;
        Eigen::Vector2d dx_beta = A_beta * x_beta + B_beta * inputData[i];
        x_beta += dt * dx_beta;
        outputData[i] = C_beta * x_beta + noise_dist(gen);
    }

    QPInput qpInput;

    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer> vectorsCollectionServer
        = std::make_shared<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>();
    qpInput.setVectorsCollectionServer(vectorsCollectionServer);
    mpc.setHankleMatrices(inputData, outputData);
    Eigen::VectorXd xInit = Eigen::VectorXd::Zero(2);
    double uData = 0.0;
    double yData = 0.0;
    mpc.updateMPC(xInit, uData, yData);
    mpc.configure(parameterHandler, qpInput);
    mpc.solveMPC();
    Eigen::Vector2d artificialEq;
    int steps = 500; // number of steps
    double referenceYZero;
    parameterHandler->getParameter("referenceYAlpha", referenceYZero);
    double referenceYOne;
    parameterHandler->getParameter("referenceYBeta", referenceYOne);

    // vectors to store data for plotting
    std::vector<double> time_vec, y0_vec, y1_vec, eq0_vec, eq1_vec, ref0_vec, ref1_vec, cost_vec,
        u0_vec, u1_vec;

    // simulate the sistem and the MPC
    for (int k = 0; k < steps; ++k)
    {
        double t = k * dt;
        // Output
        x(2) += noise_dist(gen);
        Eigen::Vector2d y = C_c * x;
        mpc.updateMPC(x.segment(0, 2), u[1], y[1]);
        mpc.solveMPC();
        mpc.getControlInput(u);
        mpc.getArtificialEquilibrium(artificialEq);
        double cost = mpc.getValueFunction();

        // Store data for plotting
        time_vec.push_back(t);
        y0_vec.push_back(y(0));
        y1_vec.push_back(y(1));
        eq0_vec.push_back(artificialEq(0));
        eq1_vec.push_back(artificialEq(1));
        ref0_vec.push_back(referenceYZero);
        ref1_vec.push_back(referenceYOne);
        cost_vec.push_back(cost);
        u0_vec.push_back(u(0));
        u1_vec.push_back(u(1));

        // Forward Euler integration: x(k+1) = x(k) + dt*(A_c*x + B_c*u)
        Eigen::Vector4d dx = A_c * x + B_c * u;
        x += dt * dx;
    }

    // Write data to a file for plotting
    std::ofstream outFile("simulation_data.dat");
    if (outFile.is_open())
    {
        outFile << "# time y(0) y(1) artificialEq(0) artificialEq(1) ref(0) ref(1) cost u(0) "
                   "u(1)\n";
        for (size_t i = 0; i < time_vec.size(); ++i)
        {
            outFile << time_vec[i] << " " << y0_vec[i] << " " << y1_vec[i] << " " << eq0_vec[i]
                    << " " << eq1_vec[i] << " " << ref0_vec[i] << " " << ref1_vec[i] << " "
                    << cost_vec[i] << " " << u0_vec[i] << " " << u1_vec[i] << "\n";
        }
        outFile.close();
    } else
    {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}
