# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

import math

# Dual-gripper nearest-neighbor heuristic with batching.
#
# High-level idea:
#     - The robot can hold up to `max_load` components at once.
#     - We alternate between:
#         1) PICK phase: move to GR nodes and pick components (up to max_load).
#         2) PLACE phase: move to KH nodes and place components until the hand is empty.
#     - We stop when all KH component demands are satisfied.
#
# Internal logic:
#     - `remaining_slots_by_type`:
#         A mapping type → set(KH slots) that still need this type.
#     - PICK phase:
#         From the current node, select among available GR nodes whose type
#         is still needed. Among these, pick the nearest GR node according
#         to the edge weight and add it to `tour` and `in_hand`.
#     - PLACE phase:
#         For each component in hand, look for the nearest KH node that:
#             * still needs that component type, and
#             * is reachable from the current node.
#         Among all candidate (component, KH) pairs, choose the move with
#         minimum distance. Update `tour`, current node, and remaining demand.
#     - If the algorithm gets temporarily stuck during placement, it moves
#       to the nearest reachable KH to unblock progress.
#
# Special moves:
#     - After all KH demands are satisfied, if there is an edge from the
#       current node to `end`, we move to `end`.
#     - If there is an edge from `end` to `start`, we finally return to `start`.
def nearest_tsp_dual(
    G,
    start: str,
    end: str,
    set_kh: list,
    set_gr: list,
    gr_types: dict,
    kh_types: dict,
    max_load: int = 2,
    verbose: bool = False,
):
    def w(u, v):
        return G[u][v]["weight"] if G.has_edge(u, v) else float("inf")

    # remaining KH demand per slot (slot id) grouped by type
    remaining_slots_by_type = {}
    for kh in set_kh:
        t = kh_types.get(kh)
        if t is None:
            continue
        remaining_slots_by_type.setdefault(t, set()).add(kh)

    def total_remaining():
        return sum(len(s) for s in remaining_slots_by_type.values())

    # helpers for nice debug
    def bag_counts_bydict(in_hand_list):
        counts = {}
        for _, t in in_hand_list:
            counts[t] = counts.get(t, 0) + 1
        return counts

    def demand_counts():
        return {t: len(slots) for t, slots in remaining_slots_by_type.items()}

    tour = [start]
    current = start

    available_gr = {g for g in set_gr if g in G}  # GR pockets we haven't used
    in_hand = []  # list of tuples (gr_node, type)

    if verbose:
        print(f"[dual-NN] start={start}, end={end}, |KH|={len(set_kh)}, |GR|={len(set_gr)}")

    while total_remaining() > 0:
        # -------- PICK PHASE --------
        while len(in_hand) < max_load:
            useful_gr = [
                g for g in available_gr
                if (tg := gr_types.get(g)) in remaining_slots_by_type
                and len(remaining_slots_by_type[tg]) > 0
                and G.has_edge(current, g)
            ]
            if not useful_gr:
                break

            next_gr = min(useful_gr, key=lambda g: w(current, g))
            t = gr_types[next_gr]

            if verbose:
                print(f" pick {next_gr} (type {t}), in_hand={bag_counts_bydict(in_hand)}")
                print(f"  → {current} -> {next_gr} (w={w(current, next_gr)})")

            tour.append(next_gr)
            current = next_gr
            in_hand.append((next_gr, t))
            available_gr.remove(next_gr)

        # No items in hand and no useful GR left → done
        if not in_hand and not any(
            (gr_types.get(g) in remaining_slots_by_type and len(remaining_slots_by_type[gr_types[g]]) > 0)
            for g in available_gr
        ):
            break

        # -------- PLACE PHASE --------
        while in_hand:
            best = None  # (dist, kh_node, item_index)
            for idx, (_, t) in enumerate(in_hand):
                slots = [kh for kh in remaining_slots_by_type.get(t, []) if G.has_edge(current, kh)]
                if not slots:
                    continue
                kh_near = min(slots, key=lambda kh: w(current, kh))
                dist = w(current, kh_near)
                if best is None or dist < best[0]:
                    best = (dist, kh_near, idx)

            if best is None:
                # Move to the nearest KH (any) to unblock (should be rare)
                reachable_kh = [kh for kh in set_kh if G.has_edge(current, kh)]
                if not reachable_kh:
                    raise ValueError(f"No reachable KH from {current} while holding items.")
                kh_near = min(reachable_kh, key=lambda kh: w(current, kh))
                tour.append(kh_near)
                current = kh_near
                continue

            dist, kh_target, idx = best
            _, t = in_hand.pop(idx)

            if verbose:
                print(f" place {kh_target} (type {t}), in_hand={bag_counts_bydict(in_hand)}, rem_kh={demand_counts()}")
                print(f"  → {current} -> {kh_target} (w={dist})")

            tour.append(kh_target)
            current = kh_target
            remaining_slots_by_type[t].remove(kh_target)
            if not remaining_slots_by_type[t]:
                del remaining_slots_by_type[t]

    # go to end, then return to start if possible
    if G.has_edge(current, end):
        if verbose:
            print(f"done with KH. going {current} -> {end}")
        tour.append(end)
        current = end
    if G.has_edge(end, start):
        if verbose:
            print(f"return {end} -> {start}")
        tour.append(start)

    return tour

# Compute total travel cost and per-step details for a tour.
def total_cost(G, tour):
    cost = 0
    time_details = []
    for i in range(len(tour) - 1):
        u, v = tour[i], tour[i + 1]
        if G.has_edge(u, v):
            w = G[u][v].get('weight', math.inf)
            cost += w
            time_details.append({"from": u, "to": v, "totalTime": w})
        else:
            # unreachable step (shouldn't happen if graph was respected)
            time_details.append({"from": u, "to": v, "totalTime": math.inf, "note": "no edge"})
    return cost, time_details
