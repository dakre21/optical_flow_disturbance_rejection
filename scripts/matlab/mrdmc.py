import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp
from scipy.signal import dlsim, dlti


class DmdStruct:

    def __init__(self, X, U, T):
        self.X = X
        self.U = U
        self.T = T


class DmdResult:

    def __init__(self, A, B, level, time, mode):
        self.A = A
        self.B = B
        self.level = level
        self.time = time
        self.mode = mode


def dmd_cvx(X, X_shifted, U):
    ns = X.shape[0]
    ni = U.shape[0]

    # Variables
    A = cp.Variable((ns, ns))
    B = cp.Variable((ns, ni))

    # Parameters
    lambda_A = 1e-3
    lambda_B = 0.1

    # Residual error
    delta = X_shifted - A @ X - B @ U

    # Objective
    fro_error = cp.norm(delta, "fro")
    A_penalty = lambda_A * cp.norm1(A)
    B_penalty = lambda_B * cp.norm1(B)
    objective = cp.Minimize(fro_error + A_penalty + B_penalty)

    # Constraints
    constraints = [
        A[0, 2] == 9.81,  # Python is 0-indexed
        A[2, 1] == 1,
        A[2, 0] == 0,
        A[1, 2] == 0,
        A[2, 2] == 0,
        B[2, 0] == 0,
    ]

    # Problem definition and solve
    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.MOSEK)

    return A.value, B.value


def mrdmdc(q, res, level=0, max_level=100):
    if not len(q):
        return res

    top = q[0]
    q.pop(0)

    if max_level <= level:
        return res

    X = top.X[:, :-1]
    X_shifted = top.X[:, 1:]
    U = top.U[:, :-1]

    #Omega = np.vstack((X, U))

    #G = X_shifted @ np.linalg.pinv(Omega)

    #n = X.shape[0]
    #A = G[:, :n]
    #B = G[:, n:]

    A, B = dmd_cvx(X, X_shifted, U)

    evals, evecs = np.linalg.eig(A)
    thresh = np.pi / top.T
    for i, eig in enumerate(evals):
        if np.abs(eig) > thresh:
            res.append(DmdResult(A, B, level, top.T, i))

    X_len = int(X.shape[-1] / 2)
    U_len = int(U.shape[-1] / 2)
    q.append(DmdStruct(X[:, :X_len], U[:, :U_len], top.T / 2))
    q.append(DmdStruct(X[:, X_len:], U[:, U_len:], top.T / 2))

    return mrdmdc(q, res, level + 1)


# Define the ned_to_body_velocity conversion function
def ned_to_body_velocity(v_fixed, roll, pitch, yaw):
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    # NED to body transformation matrix
    R = np.array(
        [
            [cp * cy, cp * sy, -sp],
            [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp],
            [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp],
        ]
    )

    return R @ v_fixed

def rates_to_pqr(rates, roll, pitch, yaw):
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    R = np.array([
        [1, 0, -sp],
        [0, cr, sr*cp],
        [0, -sr, cr*cp]
    ])

    return R @ rates


if __name__ == "__main__":
    # Read the CSV files
    pos = pd.read_csv("posn.csv")
    imu = pd.read_csv("imun.csv")
    inputs = pd.read_csv("motr.csv")
    inputs = inputs.drop(columns=["timestamp"])

    # Merge pos and imu on TimeUS
    states = pd.merge(pos, imu, on="TimeUS")
    # states = states.drop(columns=["timestamp_pos", "timestamp_imu"])

    # Merge with inputs
    frame = pd.merge(states, inputs, on="TimeUS")

    # Remove rows where Th0 == 0
    frame = frame[frame["Th0"] != 0].copy()

    # Uncomment if needed to filter by time range
    # frame = frame[frame['TimeUS'] > 100e6]
    # frame = frame[frame['TimeUS'] < 60e6]

    time = frame["TimeUS"].values
    frame = frame.drop(columns=["TimeUS"])

    # Reorder columns
    order = [
        "Th0",
        "UR",
        "UP",
        "UY",
        "X",
        "U",
        "Y",
        "V",
        "Z",
        "W",
        "Roll",
        "P",
        "Pitch",
        "Q",
        "Yaw",
        "R",
    ]
    frame = frame[order]

    # Apply velocity transformation
    for t in range(len(frame)):
        v_fixed = np.array([frame.iloc[t]["U"], frame.iloc[t]["V"], frame.iloc[t]["W"]])
        roll = frame.iloc[t]["Roll"]
        pitch = frame.iloc[t]["Pitch"]
        yaw = frame.iloc[t]["Yaw"]
        v_body = ned_to_body_velocity(v_fixed, roll, pitch, yaw)
        frame.iloc[t, frame.columns.get_loc("U")] = v_body[0]
        frame.iloc[t, frame.columns.get_loc("V")] = v_body[1]
        frame.iloc[t, frame.columns.get_loc("W")] = v_body[2]
        rates = np.array([frame.iloc[t]["P"], frame.iloc[t]["Q"], frame.iloc[t]["R"]])
        pqr = rates_to_pqr(rates, roll, pitch, yaw)
        frame.iloc[t, frame.columns.get_loc("P")] = pqr[0]
        frame.iloc[t, frame.columns.get_loc("Q")] = pqr[1]
        frame.iloc[t, frame.columns.get_loc("R")] = pqr[2]

    # Drop unused columns
    frame = frame.drop(
        columns=["Th0", "UP", "UY", "X", "U", "Y", "Z", "W", "Pitch", "Q", "Yaw", "R"]
    )

    # Keep only input and states of interest
    fr = frame[["UR", "V", "P", "Roll"]]
    frame_array = fr.to_numpy()

    # Split into inputs and states
    n_inputs = 1
    n_states = 3
    inputs = frame_array[:, :n_inputs]
    states = frame_array[:, n_inputs:]

    assert states.shape[1] == n_states
    assert inputs.shape[1] == n_inputs

    # Compute average timestep (convert from microseconds to seconds)
    dt = np.mean(np.diff(time)) * 1e-6

    # Normalize states
    mx = np.max(states, axis=0)
    states = states / mx

    T = time[-1] - time[0]
    q = [DmdStruct(states.T, inputs.T, T)]
    res = mrdmdc(q, [])

    for r in res:
        print(f"Level {r.level}")
        print(f"Time {r.time}")
        print(f"Mode {r.mode}")
        print(f"A {r.A}")
        print(f"B {r.B}")

        A = r.A
        B = r.B

        evals, evecs = np.linalg.eig(A)

        if np.any(np.abs(evals) > 1):
            continue

        C = np.array([[0, 1, 0], [0, 0, 1]])
        D = np.zeros((C.shape[0], B.shape[-1]))
        print(A.shape, B.shape, C.shape, D.shape)

        sys = dlti(A, B, C, D, dt=dt)

        t_out, x_out, y_out = dlsim(sys, inputs)

        state_labels = ["v (m/s)", "p (rad/s)", "phi (rad)"]  # Customize if needed

        n_states = x_out.shape[1]

        fig, axes = plt.subplots(n_states, 1, figsize=(10, 2.5 * n_states), sharex=True)

        for i in range(n_states):
            axes[i].plot(t_out, x_out[:, i], "r--", label="Simulated")
            axes[i].set_ylabel(f"State $x_{i+1}$\n({state_labels[i]})")
            axes[i].grid(True)
            axes[i].legend()

        axes[-1].set_xlabel("Time (s)")
        fig.suptitle("DMDc Simulated State Trajectories", fontsize=14)
        plt.tight_layout()
        plt.show()
