# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

from time import time
from dual_preprocessing import evaluate_methods_on_data

# Run the dual-gripper optimization for a single input instance.
#
# This function:
#   - Accepts already loaded `input_data` (no file I/O).
#   - Reads configuration and `method` from `input_data["data"]`.
#   - Evaluates both the heuristic and linear methods via
#     `evaluate_methods_on_data`.
#   - Depending on `method`, either:
#       * returns only the heuristic result,
#       * returns only the linear result, or
#       * compares them (heuristic-linear) and returns the better one
#         along with the percentage improvement.
#
# Returns:
#     dict: Result dictionary with keys:
#         - "optimization_run" (bool): True if the run completed.
#         - "message" (str): Description of the run and selected method.
#         - "solutionTime" (int): Elapsed time in milliseconds.
#         - "optimization_results" (dict): Contains either:
#             * {"heuristic": ...}, or
#             * {"linear": ...}, or
#             * {<winner_method>: ..., "improvement_percentage": float}
#
# Raises:
#     ValueError: If `input_data` is missing or the method is unknown.
def run_dual_optimization(
    input_data: dict,
    use_prune: bool,
    add_bias: int,
    allow_cross_rack: bool,
    cross_gr_extra_bias: int,
):
    t0 = int(time() * 1000)

    if not input_data or "data" not in input_data:
        raise ValueError("run_dual_optimization requires input_data={'data':...}")

    data = input_data["data"]
    method = data.get("method", "heuristic")

    # supported methods
    if method not in {"heuristic", "linear", "heuristic-linear"}:
        raise ValueError(f"Unknown optimization method '{method}'.")

    # evaluate both methods (same as earlier)
    eval_res = evaluate_methods_on_data(
        data,
        require_type_match_gr_kh=False,
        use_prune=use_prune,
        add_bias=add_bias,
        allow_cross_rack=allow_cross_rack,
        cross_gr_extra_bias=cross_gr_extra_bias,
        start_node="0.0",
        end_node="0.0.0",
        verbose=False
    )

    heuristic = eval_res["heuristic"]
    linear = eval_res["linear"]

    # --------------------------
    # method selection logic
    # --------------------------
    if method == "heuristic":
        winner = {"heuristic": heuristic}

    elif method == "linear":
        winner = {"linear": linear}

    else:  # heuristic-linear (comparison)
        h_cost = heuristic["cost"]
        l_cost = linear["cost"]

        if h_cost < l_cost:
            improvement = round(((l_cost - h_cost) / l_cost) * 100, 4) if l_cost else 0
            winner = {
                "heuristic": heuristic,
                "improvement_percentage": improvement
            }
        else:
            improvement = round(((h_cost - l_cost) / h_cost) * 100, 4) if h_cost else 0
            winner = {
                "linear": linear,
                "improvement_percentage": improvement
            }

    # --------------------------
    # Final result
    # --------------------------
    out = {
        "optimization_run": True,
        "message": f"Optimization ran with method='{method}'.",
        "solutionTime": int(time() * 1000) - t0,
        "optimization_results": winner
    }
    return out
