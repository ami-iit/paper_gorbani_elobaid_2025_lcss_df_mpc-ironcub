import numpy as np
import hdf5storage

class RobotLoggerPython:
    """
    A class to create and save data in a specific MATLAB struct format,
    supporting standard MAT v5 (via scipy.io) and v7.3 (via hdf5storage library).
    """

    def __init__(self,
                 list_of_descriptions,
                 yarp_robot_name_value="iRonCub-Mk3"
                 ):
        """
        Initializes the logger with a list of joint descriptions and YARP robot name.
        :param list_of_descriptions: List of joint names (strings).
        :param yarp_robot_name_value: Name of the YARP robot (string).
        :param mat_file_version: Version of the MAT file format ('5' or '7.3').
        """
        if not isinstance(list_of_descriptions, list):
             raise TypeError("list_of_descriptions must be a list.")
        if list_of_descriptions and not all(isinstance(s, str) for s in list_of_descriptions):
            raise ValueError("All items in list_of_descriptions must be strings if list is not empty.")

        if not list_of_descriptions:
            raise ValueError("Description list with joints name is empty. Please fill it with joint names.")
            self.N = None
        else:
            self.N = len(list_of_descriptions)

        self.main_struct_matlab_name = "robot_logger_device"
        self._description_field_name = "description_list"
        self._yarp_robot_name_field = "yarp_robot_name"
        self._main_struct_data = {}
        self._description_list = list_of_descriptions

        if list_of_descriptions:
            # hdf5storage handles Python lists of strings correctly for cell arrays.
            # Forcing N x 1 via NumPy object array is still good for consistency.
            description_cell_np = np.array(list_of_descriptions, dtype=object)
            if description_cell_np.ndim == 1: # Ensure N x 1
                description_cell_np = description_cell_np.reshape(-1, 1)
            self._main_struct_data[self._description_field_name] = description_cell_np

        self._add_string_char(self._yarp_robot_name_field, yarp_robot_name_value)

    def _add_string_char(self, field_name, string_value):
        if not isinstance(string_value, str):
            raise TypeError(f"Value for field '{field_name}' must be a string.")
        # ... (rest of checks as before) ...
        if field_name == self._description_field_name and self._description_field_name in self._main_struct_data :
            raise ValueError(f"Cannot use reserved field name '{self._description_field_name}' for a simple string if it's already a cell array.")
        if field_name in self._main_struct_data and not isinstance(self._main_struct_data[field_name], str):
            if isinstance(self._main_struct_data[field_name], (dict, np.ndarray)):
                raise ValueError(f"Field '{field_name}' already exists as a complex type. Cannot overwrite with simple string.")
        if field_name in self._main_struct_data and isinstance(self._main_struct_data[field_name], str):
            print(f"Warning: Overwriting existing string field '{field_name}'.")

        self._main_struct_data[field_name] = string_value # hdf5storage handles Python str -> MATLAB char

    def _traverse_and_get_parent_dict(self, field_path_parts):
        current_level_dict = self._main_struct_data
        for key_part in field_path_parts[:-1]:
            is_description_field_as_cell = (key_part == self._description_field_name and
                                           isinstance(self._main_struct_data.get(key_part), np.ndarray) and
                                           self._main_struct_data.get(key_part).dtype == object)
            is_simple_string_field = isinstance(current_level_dict.get(key_part), str)
            if is_description_field_as_cell or is_simple_string_field:
                 raise ValueError(
                    f"Cannot create nested structure under '{key_part}'. It is a reserved cell array or simple string field."
                )
            if key_part not in current_level_dict:
                current_level_dict[key_part] = {}
            elif not isinstance(current_level_dict[key_part], dict):
                raise ValueError(
                    f"Path component '{key_part}' in '{'.'.join(field_path_parts)}' "
                    f"already exists and is not a dictionary (substruct group)."
                )
            current_level_dict = current_level_dict[key_part]
        return current_level_dict

    def add_joint_positions(self, joint_positions, timestamps):
        self.add_data_series(
            field_path="joints_state.positions",
            data_values=joint_positions,
            timestamps_values=timestamps,
            elements_name=self._description_list
        )

    def add_data_series(self,
                        field_path,
                        data_values,
                        timestamps_values,
                        elements_name=None):
        """
        Adds a data series to the main struct under the specified field path.
        :param field_path: A string representing the path to the data series
                            (e.g., "joints_state.positions").
        :param data_values: A NumPy array of shape (N, M) or (M,) representing the data.
        :param timestamps_values: A NumPy array of shape (1, M) or (M,) representing the timestamps.
        :param elements_name: A list of strings representing the names of the elements in the data series.
        :raises: ValueError if the field_path is invalid or if data_values and timestamps_values do not match.
        """
        
        if not isinstance(data_values, np.ndarray):
            raise TypeError("data_values must be a NumPy array.")
        if data_values.ndim == 1:
            current_M_data = data_values.shape[0] if data_values.ndim == 1 else 1
            data_values = data_values.reshape(-1, current_M_data)  # Ensure at least 2D
            current_N = 1
        elif data_values.ndim == 2:
            current_N, current_M_data = data_values.shape
        else:
            raise ValueError(f"data_values for '{field_path}' must be a 2D (N x M) array, got {data_values.ndim}D.")

        if not isinstance(timestamps_values, np.ndarray):
            raise TypeError("timestamps_values must be a NumPy array.")
        if timestamps_values.ndim == 1:
            timestamps_values_reshaped = timestamps_values.reshape(1, -1) 
        elif timestamps_values.ndim == 2 and timestamps_values.shape[0] == 1:
            timestamps_values_reshaped = timestamps_values 
        elif timestamps_values.ndim == 2 and timestamps_values.shape[1] == 1: 
            timestamps_values_reshaped = timestamps_values.T 
        else:
            raise ValueError(f"timestamps_values for '{field_path}' must be a 1D (M,) or 2D (1,M or M,1) array to become 1xM.")

        current_M_ts = timestamps_values_reshaped.shape[1]
        if current_M_data != current_M_ts:
            if current_N == current_M_ts:
                # If data is N x M and timestamps is M x 1, we can transpose data to match.
                data_values = data_values.T
                current_N, current_M_data = data_values.shape
                current_M_ts = timestamps_values_reshaped.shape[1]
            else:
                raise ValueError(f"Mismatch in data and timestamps dimension for '{field_path}': data size={current_N} X {current_M_data}, timestamps size={current_M_ts}.")

        # hdf5storage handles the N x 1 x M conversion for 'data' and 1 x M for 'timestamps'
        # if the input Python dict has these shapes.
        # Our target is N x 1 x M for 'data'.
        data_reshaped_for_substruct = data_values.reshape(current_N, 1, current_M_data)

        data_series_substruct = {
            "data": data_reshaped_for_substruct,
            "timestamps": timestamps_values_reshaped 
        }
        # Nested dicts become nested structs with hdf5storage.

        field_path_parts = field_path.split('.')
        if not all(field_path_parts):
            raise ValueError(f"Invalid field_path '{field_path}'.")

        is_path_description_field = (field_path == self._description_field_name and
                                     self._description_field_name in self._main_struct_data and
                                     isinstance(self._main_struct_data.get(self._description_field_name), np.ndarray))
        is_path_simple_string_field = (field_path in self._main_struct_data and
                                       isinstance(self._main_struct_data.get(field_path), str))

        if is_path_description_field or is_path_simple_string_field :
            raise ValueError(f"Cannot use '{field_path}' for a data series. It's a reserved cell array or simple string field.")

        target_parent_dict = self._traverse_and_get_parent_dict(field_path_parts)
        final_key = field_path_parts[-1]
        if final_key in target_parent_dict:
            print(f"Warning: Overwriting field/substruct at '{field_path}'.")
        
        if elements_name is not None:
            if not isinstance(elements_name, list):
                raise TypeError("description must be a list.")
            if not all(isinstance(item, str) for item in elements_name):
                raise ValueError("All items in description must be strings.")
            if len(elements_name) != current_N:
                print(f"Warning in saving the elements names for {field_path}; the length of the list ({len(elements_name)}) must match the dimension of the data ({current_N}).")
            else:
                # Convert description list to a NumPy object array for MATLAB cell array compatibility
                description_cell_np = np.array(elements_name, dtype=object).reshape(-1, 1)
                data_series_substruct["elements_names"] = description_cell_np

        target_parent_dict[final_key] = data_series_substruct

    def save(self, output_filename, **kwargs):
        """
        Saves the main struct data to a .mat file in MATLAB format.
        :param output_filename: Name of the output .mat file.
        :param kwargs: Additional options for saving, such as compression.
        """
        can_save = False
        if self._main_struct_data:
            if self.N is not None or self._yarp_robot_name_field in self._main_struct_data or \
               any(isinstance(v,dict) for k,v in self._main_struct_data.items() if k not in [self._description_field_name, self._yarp_robot_name_field]):
                can_save = True
        if not can_save:
            print("Warning: Logger is effectively empty or N not determined without data. Nothing to save.")
            return
        
        is_metadata_only = True
        if self._main_struct_data:
            for key, value in self._main_struct_data.items():
                if key not in [self._description_field_name, self._yarp_robot_name_field] and isinstance(value, dict):
                    is_metadata_only = False
                    break
        if is_metadata_only and can_save:
             print(f"Warning: Only metadata present. Saving file with metadata only.")


        if not output_filename.endswith(".mat"):
            output_filename += ".mat"

        # The dictionary passed to hdf5storage.savemat is the mdict.
        # Its keys are the top-level MATLAB variable names.
        data_to_save_to_matfile = {
            self.main_struct_matlab_name: self._main_struct_data
        }

        try:
            # Prepare options for hdf5storage.savemat
            # See hdf5storage.savemat documentation for all options.
            options = hdf5storage.Options(
                matlab_compatible=True,
                store_python_metadata=False, # Crucial for MATLAB not to see Python types
                structured_numpy_ndarray_as_struct=True, # If you use structured arrays
                # Compression options
                compress=kwargs.get('compress', True), # Default to True if not specified by user
                compress_size_threshold=kwargs.get('compress_size_threshold', 1024), # Only compress if > 1KB
                compression_algorithm=kwargs.get('compression_algorithm', 'gzip'),
                compression_algorithm_options=kwargs.get('compression_algorithm_options', None) # e.g. for gzip, an int 0-9
            )
            
            hdf5storage.savemat(
                file_name=output_filename,
                mdict=data_to_save_to_matfile,
                format='7.3', # Explicitly v7.3
                options=options
            )

            print(f"\nSuccessfully saved data to '{output_filename}'.")

        except Exception as e:
            print(f"Error during saving .mat file: {e}")
            import traceback
            traceback.print_exc()

class RobotLogReader:
    """
    A class to read data from .mat files created by RobotLoggerPython.
    """
    def __init__(self, mat_file_path, main_struct_name="robot_logger_device"):
        """
        Initializes the reader by loading the .mat file.
        :param mat_file_path: Path to the .mat file.
        :param main_struct_name: The name of the main MATLAB struct in the file.
        """
        if not isinstance(mat_file_path, str):
            raise TypeError("mat_file_path must be a string.")
        if not mat_file_path.endswith(".mat"):
            # Attempt to load anyway, but hdf5storage might handle it or raise an error.
            print(f"Warning: File '{mat_file_path}' does not end with .mat. Attempting to load.")

        self.main_struct_name = main_struct_name
        try:
            # Load the entire .mat file content
            # hdf5storage.loadmat returns a dict where keys are variable names in the .mat file
            self._loaded_data = hdf5storage.loadmat(mat_file_path)
        except FileNotFoundError:
            raise FileNotFoundError(f"The file '{mat_file_path}' was not found.")
        except Exception as e:
            raise Exception(f"Error loading .mat file '{mat_file_path}': {e}")

        if self.main_struct_name not in self._loaded_data:
            raise ValueError(f"Main struct '{self.main_struct_name}' not found in '{mat_file_path}'. "
                             f"Available top-level variables: {list(self._loaded_data.keys())}")
        
        self._robot_logger_data = self._loaded_data[self.main_struct_name]
        
        # The loaded data for the main struct (e.g., robot_logger_device) might be a 1x1 struct array.
        # We need to access its first (and only) element to get the actual struct content (dictionary).
        if isinstance(self._robot_logger_data, np.ndarray) and self._robot_logger_data.shape == (1,1):
             # Check if it's a structured array (struct)
            if self._robot_logger_data.dtype.fields is not None:
                self._robot_logger_data = self._robot_logger_data[0,0]
            else:
                # It's a regular numpy array that happens to be 1x1, which is unexpected for the main struct.
                # Or it could be a cell array containing the struct.
                # If it's a cell array of one element containing the dict:
                if self._robot_logger_data.dtype == object and isinstance(self._robot_logger_data[0,0], dict):
                    self._robot_logger_data = self._robot_logger_data[0,0]
                # If it's a simple 1x1 array not behaving like a struct, it's an issue.
                # However, hdf5storage usually loads structs into dicts directly if they are scalar structs.
                # If it's a struct array, it becomes a numpy structured array.
                # The RobotLoggerPython saves a dict, which hdf5storage converts.
                # If main_struct_data was a simple dict, it should be loaded as such.
                # This case handles when the struct itself is wrapped in a 1x1 array by MATLAB.
                pass # Keep as is if not a struct array, though direct dict is expected.


    def get_data_series(self, field_path):
        """
        Retrieves the data and timestamps for a given field path.
        :param field_path: A string representing the path to the data series
                           (e.g., "joints_state.positions").
        :return: A tuple (data_array, timestamps_array) or (None, None) if not found.
        :raises: ValueError if the path is invalid or the target is not a data series.
        """
        if not isinstance(field_path, str) or not field_path:
            raise ValueError("field_path must be a non-empty string.")

        parts = field_path.split('.')
        current_level = self._robot_logger_data

        for i, part in enumerate(parts):
            if not isinstance(current_level, dict):
                # This can happen if a path component refers to a simple value (e.g. string, number)
                # or a numpy array that is not a struct (dict-like).
                # Or if RobotLoggerPython's _description_field_name or _yarp_robot_name_field was part of the path.
                raise ValueError(f"Path component '{part}' in '{field_path}' is not a struct (dictionary) "
                                 f"at level {' -> '.join(parts[:i])}. Current level type: {type(current_level)}")

            if part not in current_level:
                raise ValueError(f"Field or substruct '{part}' not found in path '{'.'.join(parts[:i+1])}'. "
                                 f"Available fields: {list(current_level.keys())}")
            
            current_level = current_level[part]
            
            # Handle cases where MATLAB struct arrays (Nx1 or 1xN) are loaded as NumPy object arrays
            # where each element is a dict. We typically expect scalar structs (1x1) to be loaded as dicts.
            # If it's an array of structs, and we expect a single struct, we might need to pick one.
            # However, RobotLoggerPython creates scalar structs for data series.
            # If current_level becomes a 1x1 structured array, extract the scalar struct.
            if isinstance(current_level, np.ndarray) and current_level.shape == (1,1) and current_level.dtype.fields is not None:
                current_level = current_level[0,0]


        # After traversing, current_level should be the data series substruct
        if isinstance(current_level, dict) and "data" in current_level and "timestamps" in current_level:
            data = current_level["data"]
            timestamps = current_level["timestamps"]
            
            # Data was saved as N x 1 x M, timestamps as 1 x M.
            # Let's squeeze the data to be N x M for easier use, or 1 x M if N=1.
            if isinstance(data, np.ndarray) and data.ndim == 3 and data.shape[1] == 1:
                data = np.squeeze(data, axis=1)
            
            return data, timestamps
        else:
            raise ValueError(f"Field '{field_path}' does not appear to be a valid data series "
                             f"substruct. Expected 'data' and 'timestamps' keys. "
                             f"Found keys: {list(current_level.keys()) if isinstance(current_level, dict) else type(current_level)}")

    def get_field(self, field_path):
        """
        Retrieves any field from the loaded data structure.
        This can be used to get metadata like 'description_list' or 'yarp_robot_name',
        or even entire substructs.
        :param field_path: A string representing the path to the field (e.g., "description_list").
        :return: The value of the field.
        :raises: ValueError if the path is invalid.
        """
        if not isinstance(field_path, str) or not field_path:
            raise ValueError("field_path must be a non-empty string.")

        parts = field_path.split('.')
        current_level = self._robot_logger_data
        
        for i, part in enumerate(parts):
            if not isinstance(current_level, dict):
                 # This can happen if a path component refers to a simple value (e.g. string, number)
                # or a numpy array that is not a struct (dict-like).
                raise ValueError(f"Path component '{part}' in '{field_path}' is not a struct (dictionary) "
                                 f"at level {' -> '.join(parts[:i])}. Current level type: {type(current_level)}")

            if part not in current_level:
                raise ValueError(f"Field or substruct '{part}' not found in path '{'.'.join(parts[:i+1])}'. "
                                 f"Available fields: {list(current_level.keys())}")
            current_level = current_level[part]

            # Handle scalar struct arrays
            if isinstance(current_level, np.ndarray) and current_level.shape == (1,1) and current_level.dtype.fields is not None:
                current_level = current_level[0,0]
                
        # If the field is 'description_list' and it was saved as an N x 1 object array of strings,
        # convert it to a simple list of strings for convenience.
        if field_path.endswith("description_list") and isinstance(current_level, np.ndarray) and current_level.dtype == object:
            if current_level.ndim == 2 and current_level.shape[1] == 1:
                return [item[0] for item in current_level.tolist()] # Convert N x 1 array of single-element lists
            elif current_level.ndim == 1:
                return current_level.tolist()


        return current_level

# Example Usage (optional, for testing):
if __name__ == '__main__':
    # This example assumes you have a file 'test_log.mat' created by RobotLoggerPython
    # Create a dummy logger and save a file for testing the reader
    print("--- Testing RobotLoggerPython ---")
    joint_names = ["joint1", "joint2", "joint3"]
    logger = RobotLoggerPython(list_of_descriptions=joint_names, yarp_robot_name_value="TestBot")

    # Add some dummy data
    ts = np.array([0.0, 0.1, 0.2, 0.3, 0.4])
    pos_data = np.random.rand(len(joint_names), len(ts)) # N x M
    vel_data = np.random.rand(len(joint_names), len(ts)) # N x M
    # M x N data
    force_data = np.random.rand(len(ts), 2) # M x 2 (assuming 2 force sensors)


    logger.add_data_series("kinematics.positions", pos_data, ts)
    logger.add_data_series("kinematics.velocities", vel_data, ts)
    logger.add_data_series("forces.fx", force_data, ts) # Will be transposed by logger
    
    # Test N=1 case for data
    single_sensor_data = np.random.rand(len(ts)) # M, (should become 1xM)
    logger.add_data_series("sensors.temperature", single_sensor_data, ts)


    test_mat_file = "test_robot_log.mat"
    try:
        logger.save(test_mat_file, compress=False) # Use a known filename

        print(f"\\n--- Testing RobotLogReader on '{test_mat_file}' ---")
        reader = RobotLogReader(test_mat_file)

        print("\\nReader initialized.")
        
        print("\\nFetching 'description_list':")
        desc_list = reader.get_field("description_list")
        print(f"Description List: {desc_list}, Type: {type(desc_list)}")

        print("\\nFetching 'yarp_robot_name':")
        robot_name = reader.get_field("yarp_robot_name")
        print(f"YARP Robot Name: {robot_name}, Type: {type(robot_name)}")

        print("\\nFetching 'kinematics.positions':")
        positions, timestamps_pos = reader.get_data_series("kinematics.positions")
        if positions is not None:
            print(f"Positions data shape: {positions.shape}") # Expected (N, M)
            print(f"Timestamps shape: {timestamps_pos.shape}") # Expected (1, M)
            # print("Positions data:", positions)
            # print("Timestamps:", timestamps_pos)

        print("\\nFetching 'kinematics.velocities':")
        velocities, timestamps_vel = reader.get_data_series("kinematics.velocities")
        if velocities is not None:
            print(f"Velocities data shape: {velocities.shape}")
            print(f"Timestamps shape: {timestamps_vel.shape}")

        print("\\nFetching 'forces.fx':")
        forces, timestamps_force = reader.get_data_series("forces.fx")
        if forces is not None:
            print(f"Forces data shape: {forces.shape}") # Expected (2, M) after transpose in logger
            print(f"Timestamps shape: {timestamps_force.shape}")

        print("\\nTesting non-existent field:")
        try:
            reader.get_data_series("non_existent.field")
        except ValueError as e:
            print(f"Caught expected error: {e}")

        print("\\nTesting path to non-data-series field:")
        try:
            reader.get_data_series("description_list")
        except ValueError as e:
            print(f"Caught expected error: {e}")
            
        print("\\nTesting path through a non-struct (e.g. yarp_robot_name):")
        try:
            reader.get_data_series("yarp_robot_name.something")
        except ValueError as e:
            print(f"Caught expected error: {e}")

    except Exception as e:
        print(f"An error occurred during the example usage: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up the test file
        import os
        if os.path.exists(test_mat_file):
            os.remove(test_mat_file)
            print(f"\\nCleaned up '{test_mat_file}'.")
