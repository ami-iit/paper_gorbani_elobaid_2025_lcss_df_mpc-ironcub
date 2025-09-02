import mujoco
import mujoco.viewer as viewer
from dataclasses import dataclass
import copy

from flight_ctrl_lib.bindings import Robot
from scipy.spatial.transform import Rotation
import bipedal_locomotion_framework.bindings as blf
import numpy as np
import idyntree.swig as idyn

from .nn_jet_model import JetModelTotal
import torch
from .jet_kalman_filter import EKFJetsTotal

import resolve_robotics_uri_py

CYLINDER_RADIUS = 0.025
CYLINDER_SCALE = 0.001

class MujocoSim:
    def __init__(self, param: "MujocoConfig", run_visulaization=True):
        """
        Initializes the Mujoco Simulation environment.

        Args:
            param: MujocoConfig object containing simulation parameters.
            run_visulaization: Boolean, if True, launches the MuJoCo passive viewer.
        """
        xml_model_path = resolve_robotics_uri_py.resolve_robotics_uri(param.config["mujoco_model_path"])
        if not xml_model_path.exists():
            raise RuntimeError(f"Failed to find the Mujoco model at {xml_model_path}")
        if not xml_model_path.is_file():
            raise RuntimeError(f"The Mujoco model path {xml_model_path} is not a file")
        self.model = mujoco.MjModel.from_xml_path(xml_model_path.as_posix())
        self.data = mujoco.MjData(self.model)
        self.robot = Robot()
        urdf_model_path = resolve_robotics_uri_py.resolve_robotics_uri(param.config["robot_model"])
        if not urdf_model_path.exists():
            raise RuntimeError(f"Failed to find the URDF model at {urdf_model_path}")
        param_handler = blf.parameters_handler.TomlParametersHandler()
        self._use_nn_jet_model = param.config.get("use_nn_jet_dynamics", False)
        self._simulate_noise = param.config.get("simulate_noise", False)

        # set the timestep to 1ms
        self.model.opt.timestep = 0.001
        self.cylinder_names = [ "l_arm_cylinder", "r_arm_cylinder", "l_jet_cylinder", "r_jet_cylinder"]
        self._cylinder_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in self.cylinder_names]

        if self._use_nn_jet_model:
            # Initialize the neural network model for jet dynamics
            self.jet_model_nn = JetModelTotal() # Changed from JetModelNN
            # Initialize EKF for jet thrust estimation
            P = np.eye(2) * 1e-1
            Q = np.eye(2) * 1e-1
            R = np.eye(2) * 5e-1
            self.jet_EKF = EKFJetsTotal(R, Q, P, self.model.opt.timestep)

        # get the parameters handler from the file
        if not param_handler.set_from_file(param.config["robot_config_file"]):
            raise RuntimeError("Failed to load the parameters from the file")
        
        # configure the Robot class
        # Ensure external_wrenches_list is a list of strings
        external_wrenches_list = param.config["external_wrenches_list"]
        if isinstance(external_wrenches_list, str):
            external_wrenches_list = [external_wrenches_list]
        if not isinstance(external_wrenches_list, list):
            raise TypeError("external_wrenches_list must be a list of strings")

        # Debugging output to confirm configure method invocation
        print("Invoking Robot.configure...")

        # Pass the corrected external_wrenches_list to the configure method
        if not self.robot.configure(urdf_model_path.as_posix(), param_handler, external_wrenches_list):
            raise RuntimeError("Failed to configure the robot")
        
        self._base_frame = self.robot.getFloatingBaseFrameName()
        self._base_frame_id_Mj = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self._base_frame)
        self._base_lin_vel_filtered = np.zeros(3)
        self._base_ang_vel_filtered = np.zeros(3)

        self._estimated_thrust_nn = torch.ones(4) * 10
        self._estimated_thrust_dot_nn = torch.zeros(4)
        self._throttle = torch.zeros(4)
        self._estimated_thrust = np.ones(4) * 10
        self._estimated_thrust_dot = np.zeros(4)

        dt = self.model.opt.timestep
        self._alpha_LP = dt / (2 * 3.14 * 3 * dt)

        if self._base_frame_id_Mj == -1:
            raise RuntimeError("Failed to get the base frame id in Mujoco")

        all_joint_names_list = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
        ]

        self.joint_names = [
            name for name in all_joint_names_list if name != "base_link_fixed_joint"
        ]

        self.desired_joint_pos = np.zeros(len(self.joint_names))

        self._robot_joint_names_list = self.robot.getAxesList()

        self._set_initial_joint_positions(param)
        self._set_initial_base_pose()
        if not self.update_robot_state():
            raise RuntimeError("Failed to update the robot state")
        
        idle_thrust = 10 * np.ones(4)
        self.set_thrust(idle_thrust)
        
        self.run_viz = run_visulaization
        if run_visulaization:
            self.viewer = viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None

    def step(self, n_steps=1):
        """
        Advances the simulation by a specified number of steps.

        Args:
            n_steps: The number of simulation steps to perform.
        """
        for _ in range(n_steps):
            if self._use_nn_jet_model:
                self._simulate_thrust_nn_model()
                self._estimated_thrust, self._estimated_thrust_dot = self.jet_EKF.update(self._estimated_thrust, self._estimated_thrust_dot, self._throttle.detach().numpy().astype(np.float64), self._estimated_thrust_nn.detach().numpy().astype(np.float64), self._estimated_thrust_dot_nn.detach().numpy().astype(np.float64))
                self.set_thrust(self._estimated_thrust)
            else:
                self._estimated_thrust = np.array(self.data.ctrl[-4:])
            self._compute_required_ctrl()
            mujoco.mj_step(self.model, self.data)
            if self.viewer is not None:
                self.viewer.sync()

    def is_running(self):
        """
        Checks if the simulation and viewer (if active) are still running.

        Returns:
            True if the simulation is running, False otherwise.
        """
        ok = True
        if self.viewer is not None:
            ok = self.viewer.is_running()
        return ok

    def close_viewer(self):
        """Closes the MuJoCo passive viewer if it was initialized."""
        if self.viewer is not None:
            self.viewer.close()

    def set_joint_positions(self, joint_positions):
        """
        Sets the desired joint positions for the robot's actuators.

        Args:
            joint_positions: A NumPy array of desired joint positions, matching the order
                             and number of joints in `self._robot_joint_names_list`.
        """
        if joint_positions.shape[0] != len(self._robot_joint_names_list):
            raise ValueError("joint_positions must have the same length as the number of joints")
        joint_pos_mujoco = np.zeros(26)
        for i in range(len(joint_positions)):
            idx = self._get_joint_index(self._robot_joint_names_list[i])
            joint_pos_mujoco[idx] = joint_positions[i]
        self.desired_joint_pos = joint_pos_mujoco
    
    def set_thrust(self, thrust):
        """
        Sets the thrust values for the robot's jet actuators.

        Args:
            thrust: A NumPy array or list of 4 thrust values.
        """
        self.data.ctrl[-4:] = thrust

    def set_throttle(self, throttle):
        """
        Sets the throttle command for the neural network jet model.

        Args:
            throttle: A NumPy array or list of 4 throttle values.
        """
        self._throttle = torch.tensor(throttle).float()

    def get_joint_positions(self):
        """
        Gets the current joint positions of the robot.

        Returns:
            A NumPy array of current joint positions.
        """
        joint_pos_robot = np.zeros(len(self._robot_joint_names_list))
        for i in range(len(self._robot_joint_names_list)):
            idx = self._get_joint_index(self._robot_joint_names_list[i])
            joint_pos_robot[i] = self.data.qpos[idx + 7]
        return joint_pos_robot
    
    def get_joint_velocities(self):
        """
        Gets the current joint velocities of the robot.

        Returns:
            A NumPy array of current joint velocities.
        """
        joint_vel_robot = np.zeros(len(self._robot_joint_names_list))
        for i in range(len(self._robot_joint_names_list)):
            idx = self._get_joint_index(self._robot_joint_names_list[i])
            joint_vel_robot[i] = self.data.qvel[idx + 6]
        return joint_vel_robot
    
    def get_estimated_thrust(self):
        """
        Gets the current estimated or applied thrust values.

        Returns:
            A NumPy array of 4 thrust values.
        """
        return self._estimated_thrust
    
    def get_estimated_thrust_nn(self):
        """
        Gets the thrust values estimated by the neural network model.

        Returns:
            A NumPy array of 4 thrust values.
        """
        return self._estimated_thrust_nn.detach().numpy().astype(np.float64)
    
    def get_estimated_thrust_dot(self):
        """
        Gets the current estimated rate of change of thrust.

        Returns:
            A NumPy array of 4 thrust rate values.
        """
        return self._estimated_thrust_dot
    
    def get_base_position(self):
        """
        Gets the current 3D position of the robot's base in the world frame.

        Returns:
            A NumPy array representing the (x, y, z) position.
        """
        return self.data.xpos[self._base_frame_id_Mj]
    
    def get_base_orientation(self):
        """
        Gets the current orientation of the robot's base as Euler angles (xyz) in radians.

        Returns:
            A NumPy array representing the Euler angles (roll, pitch, yaw).
        """
        return Rotation.from_matrix(self.data.xmat[self._base_frame_id_Mj].reshape(3, 3)).as_euler("xyz")
    
    def get_base_velocity(self):
        """
        Gets the current linear velocity of the robot's base in the world frame.
        Optionally adds simulated noise.

        Returns:
            A NumPy array representing the (vx, vy, vz) linear velocity.
        """
        noise = np.zeros(3)
        if self._simulate_noise:
            noise = np.random.normal(0, 0.015, 3)
        return Rotation.from_euler("xyz", self.get_base_orientation()).as_matrix() @ self.data.sensordata[-3:] + noise
    
    def get_base_angular_velocity(self):
        """
        Gets the current angular velocity of the robot's base in the world frame.
        Optionally adds simulated noise.

        Returns:
            A NumPy array representing the (wx, wy, wz) angular velocity.
        """
        noise = np.zeros(3)
        if self._simulate_noise:
            noise = np.random.normal(0, 0.015, 3)
        rot_mat = Rotation.from_euler("xyz", self.get_base_orientation()).as_matrix()
        return rot_mat @ self.data.sensordata[-6:-3] + noise
    
    def get_center_of_mass_position(self):
        """
        Gets the current position of the robot's center of mass.

        Returns:
            A NumPy array representing the (x, y, z) CoM position.
        """
        return self.data.subtree_com[0]
    
    def get_simulation_timestep(self):
        """
        Gets the simulation timestep (dt).

        Returns:
            The simulation timestep in seconds.
        """
        return self.model.opt.timestep
    
    def set_alpha_LP(self, dt, cutoff_freq):
        """
        Sets the alpha parameter for the low-pass filter used on base velocities.

        Args:
            dt: The timestep.
            cutoff_freq: The cutoff frequency for the low-pass filter.
        """
        self._alpha_LP = dt / (2 * 3.14 * cutoff_freq * dt)
    
    def update_robot_state(self):
        """
        Updates the internal iDynTree/flightCtrlPy Robot object state.
        This includes base pose, velocities, joint states, and jet thrusts.

        Returns:
            True if the state was updated successfully, False otherwise.
        """
        self._update_visualization_thrust()
        self._base_lin_vel_filtered = self._alpha_LP * self.get_base_velocity() + (1 - self._alpha_LP) * self._base_lin_vel_filtered
        self._base_ang_vel_filtered = self._alpha_LP * self.get_base_angular_velocity() + (1 - self._alpha_LP) * self._base_ang_vel_filtered
        base_pose = idyn.Transform()
        base_rot = idyn.Rotation().RPY(self.get_base_orientation()[0], self.get_base_orientation()[1], self.get_base_orientation()[2])
        base_pose.setRotation(base_rot)
        base_pose.setPosition(idyn.Position(self.get_base_position()))
        baseVelPython = np.concatenate((np.array(self.get_base_velocity()), np.array(self.get_base_angular_velocity())))
        baseVel = idyn.Twist().FromPython(baseVelPython)
        joint_pos = idyn.VectorDynSize(len(self._robot_joint_names_list))
        joint_pos_vector = self.get_joint_positions()
        for i in range(len(joint_pos_vector)):
            joint_pos.setVal(i, joint_pos_vector[i])
        joint_vel = idyn.VectorDynSize(len(self._robot_joint_names_list))
        joint_vel_vector = self.get_joint_velocities()
        for i in range(len(joint_vel_vector)):
            joint_vel.setVal(i, joint_vel_vector[i])
        jetThrustInNewton = np.array(self.get_estimated_thrust())
        zeroVector = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        extWrenchesInNewton = np.array([zeroVector])
        return self.robot.setState(base_pose, baseVel, joint_pos, joint_vel, jetThrustInNewton, extWrenchesInNewton)
    
    def _get_joint_index(self, joint_name):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name) -1
    
    def _set_initial_joint_positions(self, param: "MujocoConfig"):
        initial_joint_positions = param.config["intial_position"]

        for i in range(len(initial_joint_positions)):
            idx = self._get_joint_index(self._robot_joint_names_list[i])
            self.data.qpos[idx + 7] = np.radians(initial_joint_positions[i])
        mujoco.mj_forward(self.model, self.data)

    def _set_initial_base_pose(self):
        l_foot_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "l_sole")
        root_link_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "root_link")

        l_sole_pos = self.data.site_xpos[l_foot_site_id]  # World position of the site
        l_sole_mat = self.data.site_xmat[l_foot_site_id].reshape(
            3, 3
        )  # World orientation matrix of the site
        root_link_mat = self.data.site_xmat[root_link_site_id].reshape(3, 3)
        base_pose = copy.deepcopy(self.data.qpos[:3])
        base_quat = copy.deepcopy(self.data.qpos[3:7])
        base_mat = Rotation.from_quat(base_quat).as_matrix()
        relative_mat = l_sole_mat.T
        rpy = Rotation.from_matrix(relative_mat).as_euler("xyz")
        rpy[2] = 0
        relative_mat = Rotation.from_euler("xyz", rpy).as_matrix()
        b_h_l_sole = np.eye(4)
        b_h_l_sole[:3, :3] = relative_mat
        b_h_l_sole[:3, 3] = l_sole_pos
        l_sole_h_b = np.linalg.inv(b_h_l_sole)
        self.data.qpos[:3] = -l_sole_mat.T @ l_sole_pos
        # data.qpos[2] = base_pose[2] - l_sole_pos[2]
        quat = Rotation.from_matrix(
            relative_mat
        ).as_quat()  # Align base orientation with l_sole orientation
        self.data.qpos[3:7] = quat[[3, 0, 1, 2]]
        # Debugging output
        print("Base position set to:", self.data.qpos[:3])
        print("Base orientation set to (quaternion):", self.data.qpos[3:7])
        mujoco.mj_forward(self.model, self.data)

    def _compute_required_ctrl(self):
        self.data.ctrl[: len(self.joint_names)] = self.desired_joint_pos

    def _simulate_thrust_nn_model(self, n_steps=1):
        self._estimated_thrust_nn, self._estimated_thrust_dot_nn = self.jet_model_nn.get_state(
            self._estimated_thrust_nn, self._throttle, self.model.opt.timestep
        )

    def _update_visualization_thrust(self):
        for i in range(4):
            new_position = np.array([0, 0, CYLINDER_SCALE * self._estimated_thrust[i]])
            self.model.geom_pos[self._cylinder_ids[i]] = new_position
            self.model.geom_size[self._cylinder_ids[i]][0] = CYLINDER_RADIUS
            self.model.geom_size[self._cylinder_ids[i]][1] = CYLINDER_SCALE * self._estimated_thrust[i]
            self.model.geom_size[self._cylinder_ids[i]][2] = 0

@dataclass
class MujocoConfig:
    config: dict
