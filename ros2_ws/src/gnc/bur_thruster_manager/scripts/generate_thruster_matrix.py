import yaml
import numpy as np
import enum
import os

class CoordinateSystems(enum.IntEnum):
    NED=0   # North-East-Down
    NWU=1   # North-West-Up
    ENU=2   # East-North-Up

    def transform(matrix, coordinate_system_in=ENU, coordinate_system_out=ENU):
        if(coordinate_system_in==coordinate_system_out):
            return matrix

        T_ENU_to_X = {
            CoordinateSystems.ENU: np.eye(3),
            CoordinateSystems.NWU: np.array([[0, 1.0, 0],
                                            [-1.0, 0, 0],
                                            [0, 0, 1.0]]),
            CoordinateSystems.NED: np.array([[0, 1.0, 0],
                                            [1.0, 0, 0],
                                            [0, 0, -1.0]])
        }

        T_transform = T_ENU_to_X[coordinate_system_out] @ np.linalg.inv(T_ENU_to_X[coordinate_system_in])
        return (T_transform @ matrix.T).T

# This calculates the mapping between force-torque to thruster allocations

if __name__ == '__main__':
    # Can calculate using whatever coordinate system (NED, NWU, ENU), just be consistent
    coordinate_system_in = CoordinateSystems.ENU
    coordinate_system_out = CoordinateSystems.NWU
    output_filename = 'motor_force_config_NWU.yaml'

    # Full path to the output file
    output_path = os.path.join(os.getcwd(), f'../config/{output_filename}')

    # Thruster locations relative to CoM (all units in mm)
    CoM = np.array([0, 0, 0]) / 1000    # Location of center of mass from vehicle origin
    thruster_locations = np.array([
        [-10.4, 8.96, 2.95],
        [-9.56, 2.48, 2.18],
        [10.57, 8.99, 2.95],
        [9.7, 2.48, 2.18],
        [-10.29, -9.63, 2.95],
        [-9.52, -3.24, 2.18],
        [10.43, -9.63, 2.95],
        [9.7, -3.26, 2.18],
    ]) / 1000
    thruster_locations = thruster_locations * 1000 * (25.4) # Inches to mm :(

    # Prevents up/down thrusters from having nonzero surge/sway
    thruster_locations[:, 2] = 0

    deg = 45
    s = np.sin(np.deg2rad(deg))
    c = np.cos(np.deg2rad(deg))
    thruster_orientations = np.array([
                                    [c, s, 0],
                                    [0, 0, -1],
                                    [c, -s, 0],
                                    [0, 0, 1],
                                    [c, -s, 0],
                                    [0, 0, 1],
                                    [-c, -s, 0],
                                    [0, 0, -1]
                                    ])

    CoM = CoordinateSystems.transform(CoM, coordinate_system_in=coordinate_system_in, coordinate_system_out=coordinate_system_out)
    thruster_locations = CoordinateSystems.transform(thruster_locations, coordinate_system_in=coordinate_system_in, coordinate_system_out=coordinate_system_out)
    thruster_orientations = CoordinateSystems.transform(thruster_orientations, coordinate_system_in=coordinate_system_in, coordinate_system_out=coordinate_system_out)

    # Compute torques
    thruster_locations = thruster_locations - CoM
    torque = np.cross(thruster_locations, thruster_orientations)
    print("Torque\n", torque)

    # Stack to get Force-Torque conversion matrix
    A = np.vstack((thruster_orientations.T, torque.T))
    Ainv = np.linalg.pinv(A)
    Ainv = np.round(Ainv, 6)

    print("A\n", A)
    print("Ainv\n", Ainv)

    # Convert numpy arrays to lists
    Ainv = Ainv.tolist()

    # Restructure the Ainv matrix into the desired format
    motors_data = {}
    print(len(Ainv))
    for i in range(len(Ainv)):
        motor_data = {
            'surge': Ainv[i][0],
            'sway': Ainv[i][1],
            'heave': Ainv[i][2],
            'roll': Ainv[i][3],
            'pitch': Ainv[i][4],
            'yaw': Ainv[i][5]
        }
        motors_data[f'motor{i}'] = motor_data

    flip_motors = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

    yaml_data = {
        'thruster_manager': {
            'ros__parameters': {
                'max_force': 60.0,
                'max_torque': 40.0,
                'rate_limit': 0.3,
                'flip_motors': flip_motors,
                'num_motors': len(Ainv),
                **motors_data
            }
        }
    }

    try:
        with open(output_path, 'w') as file:
            yaml.dump(yaml_data, file, default_flow_style=False)
        print("YAML file", output_path, "has been generated.")
    except Exception as e:
        print(f"Error generating YAML file: {e}")
