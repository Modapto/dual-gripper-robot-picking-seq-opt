# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

import json, random
from time import time
from dual_preprocessing import evaluate_methods_on_data, shuffle_gr_sequence

# Run a dual-gripper simulation over multiple randomized GR configurations.
#
# This function:
#   - Accepts an already-decoded `input_data` dict (no file I/O).
#   - Evaluates the *baseline* GR configuration using both heuristic
#     and linear methods.
#   - Generates `num_random_gr_configs` alternative GR configurations
#     by shuffling container positions.
#   - For each randomized configuration, re-evaluates the methods.
#   - Tracks the best configuration based on the heuristic cost.
#   - Computes percentage improvement compared to the baseline.
#
# Raises:
#     ValueError: If input_data is missing or has no "data" key.
def run_dual_simulation(
    input_data: dict,
    use_prune: bool,
    add_bias: int,
    allow_cross_rack: bool,
    cross_gr_extra_bias: int
):
    t0 = int(time() * 1000)

    if not input_data or "data" not in input_data:
        raise ValueError("run_dual_simulation requires input_data={'data':...}")

    data = input_data["data"]

    num_trials = int(data.get("num_random_gr_configs", 10))
    seed = int(data.get("random_seed", 42))
    rng = random.Random(seed)

    # ---- Baseline evaluation ----
    base_eval = evaluate_methods_on_data(
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

    baseline = {
        "heuristic": {"cost": base_eval["heuristic"]["cost"]},
        "linear": {"cost": base_eval["linear"]["cost"]},
        "gr_sequence": data["gr_sequence"]
    }

    # Best tracking
    best = {
        "phase": 0,
        "heuristic": base_eval["heuristic"],
        "linear": base_eval["linear"],
        "gr_sequence": data["gr_sequence"],
        "heuristic_cost": base_eval["heuristic"]["cost"],
        "linear_cost": base_eval["linear"]["cost"]
    }

    # ---- Try all random phases ----
    for k in range(num_trials):
        tmp = json.loads(json.dumps(data))
        tmp["gr_sequence"] = shuffle_gr_sequence(tmp["gr_sequence"], rng)

        ev = evaluate_methods_on_data(
            tmp,
            require_type_match_gr_kh=False,
            use_prune=use_prune,
            add_bias=add_bias,
            allow_cross_rack=allow_cross_rack,
            cross_gr_extra_bias=cross_gr_extra_bias,
            start_node="0.0",
            end_node="0.0.0",
            verbose=False
        )

        # Compare heuristic only
        if ev["heuristic"]["cost"] < best["heuristic_cost"]:
            best = {
                "phase": k + 1,
                "heuristic": ev["heuristic"],
                "linear": ev["linear"],
                "gr_sequence": tmp["gr_sequence"],
                "heuristic_cost": ev["heuristic"]["cost"],
                "linear_cost": ev["linear"]["cost"]
            }

    # Compute improvement %
    h_base = baseline["heuristic"]["cost"]
    l_base = baseline["linear"]["cost"]

    h_best = best["heuristic_cost"]
    l_best = best["linear_cost"]

    heuristic_improvement = round(((h_base - h_best) / h_base) * 100, 4) if h_base else 0
    linear_improvement = round(((l_base - l_best) / l_base) * 100, 4) if l_base else 0

    # Final output
    out = {
        "simulation_run": True,
        "message": "Improved GR configuration found." if best["phase"] > 0 else
                   "No better GR configuration found.",
        "baseline": baseline,
        "best_phase": {
            "phase": best["phase"],
            "exact_cost": best["heuristic_cost"],
            "improvement_exact": heuristic_improvement,
            "improvement_linear": linear_improvement,
            "gr_sequence": best["gr_sequence"]
        },
        "solutionTime": int(time() * 1000) - t0,
        "totalTime": int(time() * 1000) - t0
    }

    return out
