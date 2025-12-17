# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

import json
import sys

import dual_optimization as opt
import dual_simulation as sim

DEFAULT_JSON = "input_dual.json"

# GLOBAL CONFIGS CENTRALIZED HERE
CONFIG = {
    "use_prune": True,
    "add_bias": 2000,
    "allow_cross_rack": True,
    "cross_gr_extra_bias": 0
}

ALLOWED_OPT_METHODS = {"heuristic", "linear", "heuristic-linear"}
SIMULATION_METHOD = "simulation"


# Write a Python object as JSON to the given file path.
def write_output(path, data):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Wrote {path}")

# Route a single dual-gripper request to either optimization or simulation.
#
# The function:
#     - Reads the `method` field from the `data` block (or from `msg`).
#     - If method == "simulation", calls `run_dual_simulation`.
#     - If method is in ALLOWED_OPT_METHODS, calls `run_dual_optimization`.
#     - Writes the corresponding JSON output file.
#     - Returns the result dictionary from the chosen routine.
def run_from_msg(msg: dict):
    data = msg.get("data", msg)
    method = data.get("method", "heuristic")

    cfg = CONFIG

    # ----- Simulation -----
    if method == SIMULATION_METHOD:
        print("→ Running simulation…")
        result = sim.run_dual_simulation(
            input_data={"data": data},
            **cfg
        )
        write_output("dual_simulation_output.json", result)
        return result

    # ----- Optimization -----
    if method in ALLOWED_OPT_METHODS:
        print(f"→ Running optimization method '{method}'…")
        result = opt.run_dual_optimization(
            input_data={"data": data},
            **cfg
        )
        write_output("dual_optimization_output.json", result)
        return result

    raise ValueError(f"Unknown method '{method}'.")

# Command-line entry point for dual-gripper optimization/simulation.
#
# Behavior:
#     - If no JSON path is provided via command-line, uses DEFAULT_JSON.
#     - Loads the JSON file into a Python dict.
#     - Passes the message to `run_from_msg` for processing.
#
# Command-line usage:
#     python main.py [optional_input_path]
#
# Where:
#     optional_input_path (str, optional):
#         Path to a JSON file containing the "data" block and `method`.
#         If omitted, DEFAULT_JSON is used.
def main():
    if len(sys.argv) < 2:
        json_path = DEFAULT_JSON
        print(f"(no arg given) → using default JSON: {json_path}")
    else:
        json_path = sys.argv[1]

    with open(json_path, "r") as f:
        msg = json.load(f)

    run_from_msg(msg)


if __name__ == "__main__":
    main()
