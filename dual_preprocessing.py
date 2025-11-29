# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

from collections import Counter, defaultdict
from parse_json_dual import create_distance_matrices
from GraphCreation_dual import create_directed_bipartite_graph
from heuristic_method_dual import nearest_tsp_dual
from linear_method_dual import linear_picking_dual

def parse_gr_node(n: str):
    """
    Parse a GR node name of the form 'row.col.index'.
    """
    a, b, c = n.split(".")
    return f"{a}.{b}", int(c)

def prune_gr_nodes_to_demand(gr_nodes: dict, kh_nodes: dict) -> dict:
    """
    Prune GR nodes based on KH demand, keeping only enough components per type.

    Logic:
        - Count required components per type from `kh_nodes`.
        - Sort GR nodes by (type, base position, index).
        - For each type, keep only as many GR nodes as needed to satisfy demand.
    """
    demand = Counter(kh_nodes.values())
    rows = []
    for node, comp_type in gr_nodes.items():
        base, pos = parse_gr_node(node)
        rows.append((comp_type, base, pos, node))
    rows.sort(key=lambda x: (x[0], x[1], x[2]))
    keep = set(); taken = Counter()
    for t, base, pos, node in rows:
        need = demand.get(t, 0)
        if need <= 0: 
            continue
        if taken[t] < need:
            keep.add(node); taken[t] += 1
    return {n: gr_nodes[n] for n in keep}

def generate_kh_nodes(kh_sequences, kit_holder_types):
    """
    Build KH node map from sequences and KH templates.

    Parameters:
        kh_sequences (list[list[dict]]):
            List of KH phase sequences with position→KH ID mappings.
        kit_holder_types (dict):
            Template of KH types and their contents.
    """
    kh_nodes = {}
    for entry in kh_sequences:
        for kh_pos, kh_id in entry.items():
            if kh_id in kit_holder_types:
                for comp in kit_holder_types[kh_id].get("contents", []):
                    if isinstance(comp, dict):
                        position = comp["position"]
                        comp_type = comp["type"]
                        kh_nodes[f"{kh_pos}.{position}"] = comp_type
    return kh_nodes

def generate_gr_nodes(gr_sequence, container_types):
    """
    Build GR node map from a GR sequence and container templates.

    Parameters:
        gr_sequence (list[dict]): List of {gr_pos: container_id} dictionaries.
        container_types (dict): Template of containers and their contents.
    """
    gr_nodes = {}
    for entry in gr_sequence:
        for gr_pos, container_id in entry.items():
            if container_id in container_types:
                contents = container_types[container_id].get("contents", [])
                for comp in contents:
                    if isinstance(comp, dict) and "position" in comp and "type" in comp:
                        position = comp["position"]
                        comp_type = comp["type"]
                        gr_nodes[f"{gr_pos}.{position}"] = comp_type
    return gr_nodes

def _parse_gr(node):
    """
    Internal helper to parse a GR node string.

    Parameters:
        node (str): Node name, e.g. '1.2.3'.
    """
    parts = node.split(".")
    if len(parts) == 3:
        return parts[0], parts[1], parts[2]
    return None, None, None

def _get_base(name):
    """
    Internal helper to get the 'base' part of a node name.

    - For 'x.y'   → returns 'x'
    - For 'x.y.z' → returns 'x.y'
    - Otherwise   → returns the name as-is.
    """
    parts = name.split(".")
    if len(parts) == 2:
        return parts[0]
    if len(parts) == 3:
        return ".".join(parts[:2])
    return name

def _build_lookup(min_edges):
    """
    Build a lookup dict from a minimal edge list.

    Parameters:
        min_edges (list[dict]): Each entry has keys:
            - "edge": "(src, dst)"
            - "distance": numeric distance.
    """
    return {
        tuple(e["edge"].strip("()").replace("'", "").split(", ")): e["distance"]
        for e in min_edges
    }

def _get_distance(src, dst, lookup, default=10**9):
    """
    Smart fallback distance lookup between src and dst.

    Order of attempts:
        1. Exact (src, dst).
        2. Same base as src, exact dst.
        3. Exact src, base-equivalent dst.
        4. Base-equivalent src and dst.
    """
    if (src, dst) in lookup:
        return lookup[(src, dst)]
    bs, bd = _get_base(src), _get_base(dst)
    for (s, d), w in lookup.items():
        if _get_base(s) == bs and d == dst:
            return w
    for (s, d), w in lookup.items():
        if s == src and _get_base(d) == bd:
            return w
    for (s, d), w in lookup.items():
        if _get_base(s) == bs and _get_base(d) == bd:
            return w
    return default

def extend_distance_matrix_dual(
    gr_nodes,
    kh_nodes,
    kh_sequences_opt,
    min_edges,
    add_bias=2000,
    allow_cross_rack_gr_pick2=True,
    cross_gr_extra_bias=0,
    require_type_match_gr_kh=True,
):
    """
     Build a dual-gripper distance matrix (edge list) from minimal edges.

     Constructs all necessary edges for the dual-gripper BTSP, including:
         - 0.0        → GR (first-slot only, *.1).
         - GR         → KH (optionally type-matching).
         - KH         → GR (all).
         - KH         → 0.0.0.
         - 0.0.0      → 0.0.
         - KH         → KH (no self loops).
         - GR         → GR within same rack (next index only).
         - GR         → GR across different racks (optional cross-rack moves).

     Distances are derived from `min_edges` using `_get_distance`, then
     `add_bias` and `cross_gr_extra_bias` are applied where appropriate.
     """
    lookup = _build_lookup(min_edges)
    BIG = 10**8
    out = []

    gr_set = set(gr_nodes.keys())
    kh_set = set(kh_nodes.keys())

    # 0.0 -> GR (only *.1)
    for g in gr_set:
        r, c, i = _parse_gr(g)
        if i != "1":
            continue
        w = _get_distance("0.0", g, lookup)
        if w < BIG:
            out.append({"edge": f"(0.0, {g})", "distance": w + add_bias})

    # GR -> KH (optionally require type match)
    for g in gr_set:
        for kh in kh_set:
            if require_type_match_gr_kh:
                if gr_nodes[g] != kh_nodes[kh]:
                    continue
            w = _get_distance(g, kh, lookup)
            if w < BIG:
                out.append({"edge": f"({g}, {kh})", "distance": w + add_bias})

    # KH -> GR (all)
    for kh in kh_set:
        for g in gr_set:
            w = _get_distance(kh, g, lookup)
            if w < BIG:
                out.append({"edge": f"({kh}, {g})", "distance": w + add_bias})

    # KH -> 0.0.0
    for kh in kh_set:
        w = _get_distance(kh, "0.0.0", lookup)
        if w < BIG:
            out.append({"edge": f"({kh}, 0.0.0)", "distance": w + add_bias})

    # 0.0.0 -> 0.0
    w = _get_distance("0.0.0", "0.0", lookup)
    if w < BIG:
        out.append({"edge": "(0.0.0, 0.0)", "distance": w})

    # KH -> KH (no self)
    kh_faces = list(kh_set)
    for u in kh_faces:
        for v in kh_faces:
            if u == v:
                continue
            w = _get_distance(u, v, lookup)
            if w < BIG:
                out.append({"edge": f"({u}, {v})", "distance": w})

    # GR -> GR: (a) same-base next-index hops only
    by_base = defaultdict(set)
    for g in gr_set:
        r, c, i = _parse_gr(g)
        if r is None:
            continue
        by_base[f"{r}.{c}"].add(int(i))
    for base, idxs in by_base.items():
        for k in sorted(idxs):
            nxt = k + 1
            if nxt in idxs:
                u = f"{base}.{k}"
                v = f"{base}.{nxt}"
                w = _get_distance(u, v, lookup)
                if w < BIG:
                    out.append({"edge": f"({u}, {v})", "distance": w})

    # (b) cross-rack GR→GR (never within the same base)
    if allow_cross_rack_gr_pick2:
        gr_list = list(gr_set)
        for u in gr_list:
            ru, cu, _ = _parse_gr(u)
            if ru is None:
                continue
            for v in gr_list:
                if u == v:
                    continue
                rv, cv, _ = _parse_gr(v)
                if rv is None:
                    continue
                if (ru, cu) == (rv, cv):
                    continue
                w = _get_distance(u, v, lookup)
                if w < BIG:
                    out.append({"edge": f"({u}, {v})", "distance": w + cross_gr_extra_bias})

    cleaned = []
    for e in out:
        s, d = e["edge"].strip("()").split(", ")
        if s != d:
            cleaned.append(e)
    return cleaned

def build_edges_from_input(
    data,
    use_prune=True,
    require_type_match_gr_kh=True,
    add_bias=2000,
    allow_cross_rack=True,
    cross_gr_extra_bias=0
):
    """
    Build extended edges and node-type dictionaries from raw input data.

    Steps:
        - Resolve templates (containers/KHs/KH sequences).
        - Build full GR and KH node maps.
        - Optionally prune GR nodes to KH demand.
        - Call `extend_distance_matrix_dual` to generate a full edge list.
    """
    templates = data.get("templates", {})
    containers = templates.get("containers_opt", data.get("containers_opt", {}))
    kit_holders = templates.get("kit_holders_opt", data.get("kit_holders_opt", {}))
    kh_sequences_opt = templates.get("kh_sequences_opt", data.get("kh_sequences_opt", []))
    min_edges = data.get("distance_matrix_min", templates.get("distance_matrix_min", []))

    gr_nodes_full = generate_gr_nodes(data["gr_sequence"], containers)
    kh_nodes = generate_kh_nodes(kh_sequences_opt, kit_holders)

    gr_nodes = prune_gr_nodes_to_demand(gr_nodes_full, kh_nodes) if use_prune else gr_nodes_full

    edges = extend_distance_matrix_dual(
        gr_nodes=gr_nodes,
        kh_nodes=kh_nodes,
        kh_sequences_opt=kh_sequences_opt,
        min_edges=min_edges,
        add_bias=add_bias,
        allow_cross_rack_gr_pick2=allow_cross_rack,
        cross_gr_extra_bias=cross_gr_extra_bias,
        require_type_match_gr_kh=require_type_match_gr_kh
    )
    return edges, gr_nodes, kh_nodes

def create_graph_from_edges(edges, data):
    """
    Create a directed bipartite graph from extended edges and input data.

    Steps:
        - Convert `edges` to a distance matrix.
        - Rebuild active KH nodes from templates/sequences.
        - Validate that all active KH nodes are present in the matrix.
        - Build a directed bipartite graph via `create_directed_bipartite_graph`.
    """
    a_to_b_matrix = create_distance_matrices({"data": {"distance_matrix": edges}})
    templates = data.get("templates", {})
    kit_holders_tpl = templates.get("kit_holders_opt", data.get("kit_holders_opt", {}))
    kh_seq = templates.get("kh_sequences_opt", data.get("kh_sequences_opt", []))
    kh_nodes_again = generate_kh_nodes(kh_seq, kit_holders_tpl)
    active_kh_nodes = sorted(kh_nodes_again.keys())

    missing = [n for n in active_kh_nodes if n not in a_to_b_matrix.index or n not in a_to_b_matrix.columns]
    if missing:
        raise ValueError(f"KH nodes not in matrix: {missing}")

    B, set_1, set_2 = create_directed_bipartite_graph(a_to_b_matrix, active_kh_nodes)
    return B, set_1, set_2

def node_kind(node, gr_types, kh_types):
    """
    Classify a node into one of: GR, KH, HOME, OTHER.
    """
    if node in gr_types:
        return "GR"
    if node in kh_types:
        return "KH"
    if node in ("0.0", "0.0.0"):
        return "HOME"
    return "OTHER"

def compute_path_cost(G, tour):
    """
    Compute total cost of a path/tour on graph G.
    """
    cost = 0
    for i in range(len(tour) - 1):
        u, v = tour[i], tour[i + 1]
        if (u in G) and (v in G[u]):
            cost += int(G[u][v]["weight"])
        else:
            raise ValueError(f"Edge {u} -> {v} missing in graph.")
    return cost

def build_time_details_from_tour(G, tour, gr_types, kh_types):
    """
    Build enriched time details for a tour, including component info.

    For each edge (u → v):
        - "distance" is taken from G[u][v]['weight'].
        - "component_picked" is set when v is a GR node.
        - "component_placed" is set when v is a KH node.
    """
    rows = []
    for i in range(len(tour) - 1):
        u, v = tour[i], tour[i + 1]
        dist = int(G[u][v]["weight"])
        kind = node_kind(v, gr_types, kh_types)
        row = {"from": u, "to": v, "distance": dist}
        if kind == "GR":
            row["component_picked"] = gr_types[v]
        elif kind == "KH":
            row["component_placed"] = kh_types[v]
        rows.append(row)
    return rows

def evaluate_methods_on_data(
    data,
    *,
    require_type_match_gr_kh=False,
    use_prune=True,
    add_bias=2000,
    allow_cross_rack=True,
    cross_gr_extra_bias=0,
    start_node="0.0",
    end_node="0.0.0",
    verbose=False
):
    """
    Evaluate both heuristic and linear methods on a single data instance.

    Workflow:
        1. Build edges and node-type maps via `build_edges_from_input`.
        2. Create directed bipartite graph from edges.
        3. Run:
            - `nearest_tsp_dual` (heuristic).
            - `linear_picking_dual` (linear).
        4. Build time details for each method's tour.
    """
    edges, gr_types, kh_types = build_edges_from_input(
        data,
        use_prune=use_prune,
        require_type_match_gr_kh=require_type_match_gr_kh,
        add_bias=add_bias,
        allow_cross_rack=allow_cross_rack,
        cross_gr_extra_bias=cross_gr_extra_bias
    )
    B, set_1, set_2 = create_graph_from_edges(edges, data)

    # heuristic
    tour_h = nearest_tsp_dual(
        G=B,
        start=start_node,
        end=end_node,
        set_kh=set_1,
        set_gr=set_2,
        gr_types=gr_types,
        kh_types=kh_types,
        max_load=2,
        verbose=verbose
    )
    cost_h = compute_path_cost(B, tour_h)
    td_h   = build_time_details_from_tour(B, tour_h, gr_types, kh_types)

    # linear
    lin = linear_picking_dual(
        B=B,
        start_node=start_node,
        end_node=end_node,
        set_kh=set_1,
        set_gr=set_2,
        gr_types=gr_types,
        kh_types=kh_types,
        filtered_matrix=edges
    )
    tour_l = lin["tour"]
    cost_l = lin.get("total_cost", compute_path_cost(B, tour_l))
    td_l   = build_time_details_from_tour(B, tour_l, gr_types, kh_types)

    return {
        "heuristic": {"cost": cost_h, "tour": tour_h, "time_details": td_h},
        "linear":    {"cost": cost_l, "tour": tour_l, "time_details": td_l},
        "edges": edges, "gr_types": gr_types, "kh_types": kh_types
    }

def shuffle_gr_sequence(gr_sequence, rng):
    """
    Shuffle GR sequence positions while keeping the same set of containers.
    """
    positions, containers = [], []
    for d in gr_sequence:
        [(pos, cid)] = d.items()
        positions.append(pos); containers.append(cid)
    perm = containers[:]
    rng.shuffle(perm)
    return [{pos: cid} for pos, cid in zip(positions, perm)]
