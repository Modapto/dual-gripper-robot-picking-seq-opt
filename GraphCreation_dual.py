# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

import networkx as nx

# Create a directed bipartite graph for the dual-gripper setup.
#
# The function interprets the distance matrix as a bipartite graph between:
#     - set_1: KH (kit holder) nodes, including the pseudonode '0.0'
#     - set_2: GR (gravity rack) nodes, including the pseudonode '0.0.0'
#
# Node classification is based on the name format:
#     - Nodes with two dot-separated parts (e.g. "1.1") or "0.0" → KH side (set_1).
#     - Nodes with three dot-separated parts (e.g. "1.1.3") or "0.0.0" → GR side (set_2).
#
# Special edge rules:
#     - 0.0   → GR (all GR nodes except '0.0.0')
#     - KH    → 0.0.0 (all KH nodes except '0.0')
#     - 0.0.0 → 0.0   (single closing edge with fixed weight 1)
#     - KH ↔ GR       (bidirectional edges where distance < 1,000,000)
#     - KH → KH       (only between `active_kh_positions`, excluding '0.0')
#     - GR → GR       (all GR→GR edges except those involving '0.0.0')
def create_directed_bipartite_graph(a_to_b_matrix,active_kh_positions, large_number=1000000):
    B = nx.DiGraph()

    # Define node groups
    set_1 = [node for node in a_to_b_matrix.index if len(node.split('.')) == 2 or node == '0.0']
    set_2 = [node for node in a_to_b_matrix.index if len(node.split('.')) == 3 or node == '0.0.0']

    # Add nodes with bipartite attributes
    for node in set_1:
        B.add_node(node, bipartite=0)
    for node in set_2:
        B.add_node(node, bipartite=1)

    # 1. Add edges from 0.0 → GR (excluding 0.0.0)
    for gr in set_2:
        if gr != '0.0.0':
            weight = a_to_b_matrix.at['0.0', gr]
            if weight < 1000000:
                B.add_edge('0.0', gr, weight=weight)
                # print(f"Start node 0.0 connects to: {gr}")

    # 2. Add edges from KH → 0.0.0 (excluding 0.0)
    for kh in set_1:
        if kh != '0.0':
            weight = a_to_b_matrix.at[kh, '0.0.0']
            if weight < 1000000:
                B.add_edge(kh, '0.0.0', weight=weight)
                # print(f"{kh} goes to final node 0.0.0")

    # 3. Add final return edge 0.0.0 → 0.0
    B.add_edge('0.0.0', '0.0', weight=1)
    print("Final return from 0.0.0 → 0.0 confirmed")

    # 4. Add bidirectional edges between KH and GR
    for kh in set_1:
        for gr in set_2:
            if kh != '0.0' and gr != '0.0.0':
                weight_kh_to_gr = a_to_b_matrix.at[kh, gr]
                weight_gr_to_kh = a_to_b_matrix.at[gr, kh]
                if weight_kh_to_gr < 1000000:
                    B.add_edge(kh, gr, weight=weight_kh_to_gr)
                if weight_gr_to_kh < 1000000:
                    B.add_edge(gr, kh, weight=weight_gr_to_kh)
    print("KH nodes:", [n for n in set_1 if n != "0.0"])

    # Step 5: KH → KH
    for u in active_kh_positions:
        for v in active_kh_positions:
            if u != v and u != '0.0' and v != '0.0':
                weight = a_to_b_matrix.at[u, v]
                # print(f"Checking KH→KH candidate {u}->{v}: weight={weight}")
                if weight < large_number:
                    # print(f"  ✔ adding {u}->{v}")
                    B.add_edge(u, v, weight=weight)
    # 6. Add GR → GR (excluding 0.0.0)
    for u in set_2:
        for v in set_2:
            if u != v and u != "0.0.0" and v != "0.0.0":
                try:
                    weight = a_to_b_matrix.at[u, v]
                    if weight < 1000000:
                        B.add_edge(u, v, weight=weight)
                        # print(f"GR → GR edge added: {u} → {v} | weight = {weight}")
                except KeyError:
                    continue

    return B, set_1, set_2