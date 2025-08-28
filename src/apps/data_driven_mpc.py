import sys
from pathlib import Path
# Add the project root to sys.path so local modules can be imported
project_root = Path(__file__).resolve().parents[1]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

import numpy as np
if not hasattr(np, "unicode_"):
    np.unicode_ = np.str_
import toml
from mujoco_lib.ironcub_mujoco_simulator import MujocoConfig, MujocoSim
import flight_ctrl_lib.bindings as flightCtrl
from momentum_based_mpc.bindingsMPC import DataFusedMPC
import bipedal_locomotion_framework.bindings as blf
import time
import mujoco
import imageio
from mujoco_lib.input_generator import turb_identSignal_may2024
from mujoco_lib.single_jet_simulator import SingleJetSimulator as JetNNSimulator
from mujoco_lib.robot_logging import RobotLoggerPython

OUTPUT_VIDEO_PATH = 'simulation_record.mp4'
FRAME_RATE = 30       # Video frame rate (Hz)
SAVE_VIDEO = True
SAVE_DATA = False

if __name__ == "__main__":

    rf = flightCtrl.ResourceFinder()

    toml_file_path = Path(__file__).parents[0] / "config/configMujoco.toml"

    configSim = MujocoConfig(toml.load(toml_file_path))

    robot_config_file = Path(__file__).parents[0] / "config/robot.toml"
    configSim.config["robot_config_file"] = str(robot_config_file)

    xml_path = rf.findFileByName("iRonCub-Mk3_Mujoco/iRonCub.xml")
    print("xml_path:", xml_path)
    configSim.config["mujoco_model_path"] = str(xml_path)
    run_viz = False
    n_iter = 5500
    sim = MujocoSim(configSim, run_visulaization=run_viz)

    flightCtrl.useSystemClock()
    param_handler = blf.parameters_handler.YarpParametersHandler()
    rf = flightCtrl.ResourceFinder()
    configFileFullPath = str(Path(__file__).parents[0] / "config/dd_mcp_config.xml")
    print("configFileFullPath:", configFileFullPath)
    if not flightCtrl.readXMLFile(configFileFullPath, param_handler):
        raise RuntimeError("Failed to read the configuration file")
    
    param_handler_mpc = param_handler.get_group("DD_MPC_CONFIG")

    dt = 0.1
    Ts = dt
    fs = 1/Ts
    total_duration = 300  # Total duration for data generation
    time_points = np.arange(150,150 + total_duration, 1.0)
    u_signal = [turb_identSignal_may2024(t) for t in time_points]
    u_data = [np.array([u], dtype=np.float64) for u in u_signal]
    T = len(u_data)

    jetModel_offline = JetNNSimulator(0.01)
    y_data = []
    for i in range(T):
        jetModel_offline.set_throttle(u_data[i][0])
        for _ in range(10):
            jetModel_offline.advance()
        y_k = jetModel_offline.get_thrust()
        y_data.append(y_k)

    if len(u_data) != len(y_data):
        raise RuntimeError("Mismatch between u_data and y_data lengths.")

    dataInputVec = [u_data, u_data, u_data, u_data]
    dataOutputVec = [y_data, y_data, y_data, y_data]

    MPCPeriod = param_handler_mpc.get_parameter_float("periodMPC")
    sim.set_alpha_LP(MPCPeriod, 3)
    periodSim = sim.get_simulation_timestep()
    n_steps = int(MPCPeriod / periodSim)

    initial_throttle = np.zeros(4)
    estimated_thrust = sim.get_estimated_thrust()
    estimated_thrust_dot = np.zeros(4)

    qpInput = flightCtrl.QPInput()
    qpInput.setRobot(sim.robot)
    qpInput.setRobotReference(sim.robot)
    qpInput.setEmptyVectorsCollectionServer()
    qpInput.setThrottleMPC(initial_throttle)
    qpInput.setEmptyJetModel()
    qpInput.setThrustDesMPC(estimated_thrust)
    qpInput.setThrustDotDesMPC(np.zeros(4))
    qpInput.setEstimatedThrustDot(estimated_thrust_dot)
    qpInput.setOutputQPJointsPosition(sim.get_joint_positions())
    measured_joint_positions = sim.get_joint_positions()

    joint_pos_init = sim.get_joint_positions()

    data_fused_mpc = DataFusedMPC()

    if not data_fused_mpc.setHankleMatrices(dataInputVec, dataOutputVec):
        raise RuntimeError("Failed to set the Hankle matrices")

    if not data_fused_mpc.configure(param_handler_mpc, qpInput):
        raise RuntimeError("Failed to initialize the MPC")

    time_ctrl = 0.0

    if SAVE_DATA:
        joint_list = sim.robot.getAxesList()
        logging = RobotLoggerPython(joint_list)
        mom_dot = []
        CoMPosition = []
        CoMPosition_desired = []
        estimated_thrust = []
        estimated_thrust_dot = []
        thrust_desired = []
        base_position = []
        base_orientation = []
        base_orientation_desired = []
        base_lin_vel = []
        base_ang_vel = []
        base_lin_vel_filtered = []
        base_ang_vel_filtered = []
        linear_momentum = []
        angular_momentum = []
        momentum_reference = []
        alpha_gravity = []
        joint_pos_meas = []
        joint_pos_ref = []
        throttle_saved = []
        time_controller = []
        time_MPC = []
        value_function = []
        artificial_equilibrium = []
        final_CoM = []
        final_rpy = []
    
    if SAVE_VIDEO:
        render = mujoco.Renderer(sim.model, width=1920, height=1088)
        frames = []
        CoMPosition_render = []
        CoMPosition_ref_render = []
        counter_video = 0
    counter = 0
    time_sum = 0

    # while sim.is_running():
    for _ in range(n_iter):
        sim.update_robot_state()
        estim_thrust_dot = sim.get_estimated_thrust_dot()
        qpInput.setEstimatedThrustDot(estim_thrust_dot)
        tic = time.perf_counter()
        data_fused_mpc.update(qpInput)
        data_fused_mpc.solveMPC()
        toc = time.perf_counter()
        time_sum += toc - tic
        if counter == 200:
            print("average time update MPC: ", time_sum / counter)
            time_sum = 0
            counter = 0
        counter += 1

        if time_ctrl > 30 and SAVE_VIDEO:
            if counter_video == 3:
                render.update_scene(sim.data, camera="isometric")
                pixels = render.render()
                frames.append(pixels)
                counter_video = 0
            counter_video += 1
        if (toc - tic) > MPCPeriod:
            print("MPC exceeded the period by:", (toc - tic) - MPCPeriod)
        desired_thrust = data_fused_mpc.getThrustReference()
        throttle_command = data_fused_mpc.getThrottleReference()
        joint_positions_reference = data_fused_mpc.getJointsReferencePosition()
        qpInput.setThrottleMPC(throttle_command)
        qpInput.setThrustDesMPC(desired_thrust)
        qpInput.setOutputQPJointsPosition(joint_positions_reference)
        if not sim._use_nn_jet_model:
            sim.set_thrust(desired_thrust)
        sim.set_joint_positions(joint_positions_reference)
        sim.set_throttle(throttle_command)
        time_ctrl += MPCPeriod
        if SAVE_DATA:
            CoMPosition.append(np.copy(sim.robot.getPositionCoM()))
            CoMPosition_desired.append(np.copy(qpInput.getPosCoMReference()))
            estimated_thrust.append(sim.get_estimated_thrust())
            estimated_thrust_dot.append(sim.get_estimated_thrust_dot())
            thrust_desired.append(desired_thrust)
            base_position.append(sim.robot.getBasePosition())
            base_orientation.append(np.copy(sim.robot.getBaseOrientation()))
            base_orientation_desired.append(np.copy(qpInput.getRPYReference()))
            base_lin_vel.append(sim.get_base_velocity())
            base_ang_vel.append(sim.get_base_angular_velocity())
            base_lin_vel_filtered.append(sim._base_lin_vel_filtered)
            base_ang_vel_filtered.append(sim._base_ang_vel_filtered)
            linear_momentum.append(np.copy(sim.robot.getMomentum(True)[:3]))
            angular_momentum.append(np.copy(sim.robot.getMomentum(True)[3:6]))
            momentum_reference.append(qpInput.getMomentumReference())
            alpha_gravity.append(qpInput.getAlphaGravity())
            joint_pos_meas.append(sim.get_joint_positions())
            joint_pos_ref.append(joint_positions_reference)
            time_controller.append(time_ctrl)
            throttle_saved.append(throttle_command)
            mom_dot.append(sim.robot.getMatrixAmomJets() @ sim.get_estimated_thrust())
            time_MPC.append(toc - tic)
            value_function.append(data_fused_mpc.getValueFunction())
            artificial_equilibrium.append(data_fused_mpc.getArtificialEquilibrium())
            final_CoM.append(data_fused_mpc.getFinalCoMPosition())
            final_rpy.append(data_fused_mpc.getFinalRPY())
        
        sim.step(n_steps)


if SAVE_DATA:
    logging.add_joint_positions(np.array(joint_pos_meas).T, np.array(time_controller))
    logging.add_data_series("joints_state.reference_positions", np.array(joint_pos_ref).T, np.array(time_controller))
    logging.add_data_series("base_state.orientation", np.array(base_orientation).T, np.array(time_controller))
    logging.add_data_series("base_state.orientation_desired", np.array(base_orientation_desired).T, np.array(time_controller))
    logging.add_data_series("base_state.position", np.array(base_position), np.array(time_controller))
    logging.add_data_series("base_state.linear_velocity", np.array(base_lin_vel).T, np.array(time_controller))
    logging.add_data_series("base_state.angular_velocity", np.array(base_ang_vel).T, np.array(time_controller))
    logging.add_data_series("base_state.linear_velocity_filtered", np.array(base_lin_vel_filtered).T, np.array(time_controller))
    logging.add_data_series("base_state.angular_velocity_filtered", np.array(base_ang_vel_filtered).T, np.array(time_controller))
    logging.add_data_series("CoM_position", np.array(CoMPosition).T, np.array(time_controller))
    logging.add_data_series("CoM_position_desired", np.array(CoMPosition_desired).T, np.array(time_controller))
    logging.add_data_series("estimated_thrust", np.array(estimated_thrust).T, np.array(time_controller))
    logging.add_data_series("estimated_thrust_dot", np.array(estimated_thrust_dot).T, np.array(time_controller))
    logging.add_data_series("thrust_desired", np.array(thrust_desired).T, np.array(time_controller))
    logging.add_data_series("momentum.linear", np.array(linear_momentum).T, np.array(time_controller))
    logging.add_data_series("momentum.angular", np.array(angular_momentum).T, np.array(time_controller))
    logging.add_data_series("momentum_reference", np.array(momentum_reference).T, np.array(time_controller))
    logging.add_data_series("alpha_gravity", np.array(alpha_gravity), np.array(time_controller))
    logging.add_data_series("throttle", np.array(throttle_saved).T, np.array(time_controller))
    logging.add_data_series("mom_dot", np.array(mom_dot).T, np.array(time_controller))
    logging.add_data_series("time_MPC", np.array(time_MPC), np.array(time_controller))
    logging.add_data_series("value_function", np.array(value_function), np.array(time_controller))
    logging.add_data_series("artificial_equilibrium", np.array(artificial_equilibrium).T, np.array(time_controller))
    logging.add_data_series("final_CoM", np.array(final_CoM).T, np.array(time_controller))
    logging.add_data_series("final_rpy", np.array(final_rpy).T, np.array(time_controller))

    # get the current date and time
    now = time.localtime()
    current_time = time.strftime("%Y-%m-%d_%H-%M-%S", now)
    logging.save(current_time + ".mat")

# --- Save Video ---
if SAVE_VIDEO:
    if frames:
        print(f"Saving video to {OUTPUT_VIDEO_PATH} at {FRAME_RATE} FPS...")
        try:
            with imageio.get_writer(OUTPUT_VIDEO_PATH, fps=FRAME_RATE) as writer:
                for frame in frames:
                    writer.append_data(frame)
            print("Video saved successfully.")
        except Exception as e:
            print(f"Error saving video: {e}")
    else:
        print("No frames collected, video not saved.")
