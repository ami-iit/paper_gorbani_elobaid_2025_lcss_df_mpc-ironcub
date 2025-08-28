import torch

class NeuralJetModel(torch.nn.Module):
    def __init__(self, input_size, hidden_size, output_size):
        super(NeuralJetModel, self).__init__()
        self.lstm = torch.nn.LSTM(input_size, hidden_size, batch_first=True)
        self.fc = torch.nn.Linear(hidden_size, output_size)

    def forward(self, x, dt):
        # x shape: (batch, seq_len, input_size)
        lstm_out, _ = self.lstm(x) # lstm_out shape: (batch, seq_len, hidden_size)
        # Use output of the last time step for the FC layer
        out = self.fc(lstm_out[:, -1, :]) # out shape: (batch, output_size)
        # current_thrust_norm is x[:, -1, 0]
        T_next_norm = x[:, -1, 0] + out.squeeze(-1) * dt 
        return T_next_norm.unsqueeze(-1)

    def get_state(self, x, dt):
        # x shape: (batch, seq_len, input_size)
        lstm_out, _ = self.lstm(x) # lstm_out shape: (batch, seq_len, hidden_size)
        # T_dot_norm is the output of the FC layer based on the last LSTM state
        T_dot_norm = self.fc(lstm_out[:, -1, :]) # T_dot_norm shape: (batch, output_size)
        
        current_thrust_norm = x[:, -1, 0] # Get the thrust_norm from the input sequence's last step
        
        # T_next_norm is current_thrust_norm + T_dot_norm * dt
        T_next_norm = current_thrust_norm + T_dot_norm.squeeze(-1) * dt
        
        # Return T_next_norm and T_dot_norm, ensuring consistent shapes
        return T_next_norm.unsqueeze(-1), T_dot_norm

class JetModelTotal:
    def __init__(self, model_path="jet_model_torch/model_7.pth", num_jets=4): # Default path relative to this file's dir
        self.num_jets = num_jets

        # Load the neural network model ONCE
        self.neural_net = NeuralJetModel(2, 80, 1)
        
        # Construct path relative to this script file if a relative path is given
        import os
        if not os.path.isabs(model_path):
            script_dir = os.path.dirname(os.path.abspath(__file__))
            resolved_model_path = os.path.join(script_dir, model_path)
        else:
            resolved_model_path = model_path

        try:
            # Ensure the model is loaded onto CPU if no GPU is intended or available for this part
            # The checkpoint now contains both model state_dict and metadata
            full_checkpoint = torch.load(resolved_model_path, map_location=torch.device('cpu'))
        except FileNotFoundError:
            # Fallback for robustness if the script's execution context changes where __file__ points
            alt_model_path = "software/mujoco/jet_model_torch/model_7.pth" # Original hardcoded path
            print(f"Warning: Model not found at {resolved_model_path}. Trying {alt_model_path}")
            full_checkpoint = torch.load(alt_model_path, map_location=torch.device('cpu'))

        # Load model state_dict from the checkpoint
        self.neural_net.load_state_dict(full_checkpoint['model_state_dict'])
        self.neural_net.eval() # Set model to evaluation mode

        # Load metadata for normalization constants from the checkpoint
        metadata = full_checkpoint['metadata']
        self.thrust_mean = metadata['thrust_mean']
        self.thrust_std = metadata['thrust_std']
        self.throttle_mean = metadata['throttle_mean']
        self.throttle_std = metadata['throttle_std']

    def _normalize_data_single_jet(self, thrust_sj, throttle_sj):
        # thrust_sj, throttle_sj are 0-dim torch tensors or Python scalars for a single jet
        thrust_val = thrust_sj.item() if isinstance(thrust_sj, torch.Tensor) else thrust_sj
        throttle_val = throttle_sj.item() if isinstance(throttle_sj, torch.Tensor) else throttle_sj

        thrust_norm = (thrust_val - self.thrust_mean) / self.thrust_std
        throttle_norm = (throttle_val - self.throttle_mean) / self.throttle_std
        # Reshape for LSTM: (batch_size=1, seq_len=1, input_size=2)
        return torch.tensor([[[thrust_norm, throttle_norm]]], dtype=torch.float32)

    def _denormalize_thrust(self, T_norm_sj):
        # T_norm_sj is a 0-dim or 1-dim tensor for a single jet
        return T_norm_sj * self.thrust_std + self.thrust_mean

    def _denormalize_thrust_dot(self, T_dot_norm_sj):
        # T_dot_norm_sj is a 0-dim or 1-dim tensor for a single jet
        return T_dot_norm_sj * self.thrust_std

    def get_state(self, current_thrusts_all_jets, current_throttles_all_jets, dt):
        # current_thrusts_all_jets, current_throttles_all_jets are tensors of shape (num_jets,)
        # dt is a scalar
        
        next_T_all_jets = torch.zeros_like(current_thrusts_all_jets)
        next_T_dot_all_jets = torch.zeros_like(current_thrusts_all_jets)

        # Perform computations without accumulating gradients if not training
        with torch.no_grad():
            for i in range(self.num_jets):
                thrust_i = current_thrusts_all_jets[i]
                throttle_i = current_throttles_all_jets[i]
                
                normalized_input_i = self._normalize_data_single_jet(thrust_i, throttle_i) # Shape (1,1,2)
                
                # model.get_state returns (T_next_norm, T_dot_norm)
                # Each is expected to be shape (1,1) for T_next_norm and (1,1) for T_dot_norm
                T_next_norm_i, T_dot_norm_i = self.neural_net.get_state(normalized_input_i, dt)
                
                # Squeeze to make them 0-dim or scalar-like before denormalization
                next_T_all_jets[i] = self._denormalize_thrust(T_next_norm_i.squeeze())
                next_T_dot_all_jets[i] = self._denormalize_thrust_dot(T_dot_norm_i.squeeze())
            
        return next_T_all_jets, next_T_dot_all_jets
