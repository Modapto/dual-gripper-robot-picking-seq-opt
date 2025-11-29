# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

def linear_picking_dual(
    B,
    start_node: str,
    end_node: str,
    set_kh: list,
    set_gr: list,
    gr_types: dict,
    kh_types: dict,
    filtered_matrix=None,
):
    """
    Baseline dual-gripper picking heuristic (linear / greedy).

    - The robot has two grippers.
    - It tries to serve KH slots in order, pairing them as (KH1, KH2)
        when possible:
          pick → pick → place → place
    - If a second suitable GR node for KH2 is not reachable, it falls
        back to a simple:
          pick → place
        for KH1 only.
    """
    # --- helpers ---
    distance_dict = {e["edge"]: e["distance"] for e in (filtered_matrix or [])}

    def w(u, v):
        if u in B and v in B[u]:
            key = f"({u}, {v})"
            return distance_dict.get(key, B[u][v]["weight"])
        return float("inf")

    # available GR nodes by type
    gr_by_type = {}
    for g in set_gr:
        t = gr_types.get(g)
        if t is None:
            continue
        gr_by_type.setdefault(t, set()).add(g)

    # === IMPORTANT: only keep true KH nodes ===
    kh_order = [kh for kh in sorted(set_kh) if kh in kh_types]

    tour = [start_node]
    cur = start_node
    total_cost = 0

    def step(u, v):
        nonlocal total_cost, cur, tour
        dist = w(u, v)
        if dist == float("inf"):
            raise ValueError(f"No edge {u} -> {v} in graph for linear_dual baseline.")
        tour.append(v)
        total_cost += dist
        cur = v
        return dist

    i = 0
    while i < len(kh_order):
        kh1 = kh_order[i]
        # extra guard in case something odd slipped through
        if kh1 not in kh_types:
            i += 1
            continue

        t1 = kh_types[kh1]

        # nearest GR for KH1's type FROM current
        cand1 = [g for g in gr_by_type.get(t1, set()) if B.has_edge(cur, g)]
        if not cand1:
            # if none directly reachable, try any remaining of that type and let step() validate edge
            cand1 = list(gr_by_type.get(t1, set()))
            if not cand1:
                raise ValueError(f"No available GR for type {t1} for KH {kh1}")

        g1 = min(cand1, key=lambda g: w(cur, g))
        # pick #1
        step(cur, g1)
        gr_by_type[t1].remove(g1)
        if not gr_by_type[t1]:
            gr_by_type.pop(t1, None)

        # try to pair with KH2
        picked_two = False
        if i + 1 < len(kh_order):
            kh2 = kh_order[i + 1]
            if kh2 in kh_types:
                t2 = kh_types[kh2]
                cand2 = [g for g in gr_by_type.get(t2, set()) if B.has_edge(cur, g)]
                if not cand2:
                    cand2 = list(gr_by_type.get(t2, set()))
                if cand2:
                    g2 = min(cand2, key=lambda g: w(cur, g))
                    # pick #2
                    step(cur, g2)
                    gr_by_type[t2].remove(g2)
                    if not gr_by_type[t2]:
                        gr_by_type.pop(t2, None)
                    # place in KH order: KH1 then KH2
                    step(cur, kh1)
                    step(cur, kh2)
                    i += 2
                    picked_two = True

        if not picked_two:
            # Only KH1 was paired (or no KH2 left). Place KH1 now.
            step(cur, kh1)
            i += 1

    # go to end, then optionally home
    if B.has_edge(cur, end_node):
        step(cur, end_node)
        if end_node != "0.0" and B.has_edge(end_node, "0.0"):
            step(end_node, "0.0")

    # time details
    time_details = []
    for a, b in zip(tour[:-1], tour[1:]):
        rec = {"from": a, "to": b, "distance": int(w(a, b))}
        if b in gr_types:
            rec["component_picked"] = gr_types[b]
        elif b in kh_types:
            rec["component_placed"] = kh_types[b]
        time_details.append(rec)

    return {"tour": tour, "total_cost": total_cost, "time_details": time_details}
