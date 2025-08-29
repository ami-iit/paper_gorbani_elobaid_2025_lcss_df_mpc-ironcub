import torch
from .nn_jet_model import JetModelTotal

class SingleJetSimulator:
    def __init__(self, dt, initial_thrust=10.0, initial_throttle=0.0, model_path="jet_model_torch/model_7.pth"):
        """
        Initializes the single jet simulator.

        Args:
            dt (float): The simulation time step.
            initial_thrust (float, optional): The initial thrust of the turbine. Defaults to 10.0.
            initial_throttle (float, optional): The initial throttle of the turbine. Defaults to 0.0.
            model_path (str, optional): Path to the jet model file.
        """
        self.dt = dt
        # JetModelTotal expects num_jets, even if it's 1, to correctly handle tensor shapes.
        self.jet_model = JetModelTotal(model_path=model_path, num_jets=1)
        self.current_thrust = torch.tensor([initial_thrust], dtype=torch.float32)
        self.current_throttle = torch.tensor([initial_throttle], dtype=torch.float32)

    def set_throttle(self, throttle_value):
        """
        Sets the throttle for the turbine.

        Args:
            throttle_value (float): The new throttle value.
        """
        self.current_throttle = torch.tensor([throttle_value], dtype=torch.float32)

    def get_thrust(self):
        """
        Gets the current thrust of the turbine.

        Returns:
            float: The current thrust value.
        """
        return self.current_thrust.item()

    def advance(self):
        """
        Advances the simulation by one time step (dt).
        Updates the internal thrust based on the current throttle and the jet model.
        """
        # JetModelTotal.get_state expects tensors of shape (num_jets,)
        # current_thrusts_all_jets should be self.current_thrust (already a tensor [thrust_val])
        # current_throttles_all_jets should be self.current_throttle (already a tensor [throttle_val])
        
        next_thrust, _ = self.jet_model.get_state(
            self.current_thrust, 
            self.current_throttle, 
            self.dt
        )
        
        # next_thrust will be a tensor of shape (1,), so we take the first element.
        self.current_thrust = next_thrust 
        # No need to .item() here, keep it as a tensor for the next iteration's input

if __name__ == '__main__':
    # Example Usage:
    sim_dt = 0.01  # Simulation time step of 10ms
    jet_simulator = SingleJetSimulator(dt=sim_dt, initial_thrust=10.0, initial_throttle=20.0)

    print(f"Initial Thrust: {jet_simulator.get_thrust()}")

    # Simulate a few steps
    for i in range(5):
        # Change throttle at some point
        if i == 2:
            print("Setting throttle to 60.0")
            jet_simulator.set_throttle(60.0)
        
        jet_simulator.advance()
        print(f"Step {i+1}: Thrust = {jet_simulator.get_thrust():.4f}, Throttle = {jet_simulator.current_throttle.item():.1f}")

    # Example with default initial conditions
    print("\n--- New Simulation with default initial conditions ---")
    jet_simulator_default = SingleJetSimulator(dt=0.02)
    print(f"Initial Thrust: {jet_simulator_default.get_thrust()}")
    jet_simulator_default.set_throttle(50.0)
    for i in range(3):
        jet_simulator_default.advance()
        print(f"Step {i+1}: Thrust = {jet_simulator_default.get_thrust():.4f}, Throttle = {jet_simulator_default.current_throttle.item():.1f}")

