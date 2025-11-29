# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

import copy
import random

def normalize_gr_sequence(gr_sequence):
    """
    Normalize a GR sequence into a stable list of {position: container_id} dicts.

    Input can be:
        - list[dict[str, str]] like:
              [{"1.1": "Container_A"}, {"1.2": "Container_B"}, ...]
        - dict[str, str] like:
              {"1.1": "Container_A", "1.2": "Container_B", ...}

    The function:
        - Flattens the input into (position, container) pairs.
        - Sorts them by numeric (row, column) extracted from "row.col".
        - Returns a list of single-key dictionaries in that sorted order.

    This guarantees a consistent, deterministic ordering of GR positions.
    """
    if isinstance(gr_sequence, dict):
        items = list(gr_sequence.items())
    else:
        items = []
        for entry in gr_sequence:
            items.extend(entry.items())

    def rc_key(pos):
        try:
            r, c = pos.split(".")
            return (int(r), int(c))
        except Exception:
            return (10**9, pos)

    items.sort(key=lambda kv: rc_key(kv[0]))
    return [{k: v} for k, v in items]

def extract_positions_and_containers(gr_sequence):
    """
    Extract positions and container IDs from a GR sequence.

    Parameters:
        gr_sequence: GR sequence in any supported form (passed to normalize_gr_sequence).
    """
    norm = normalize_gr_sequence(gr_sequence)
    positions = [list(d.keys())[0] for d in norm]
    containers = [list(d.values())[0] for d in norm]
    return positions, containers

def shuffle_container_positions(gr_sequence, seed=None, strategy="shuffle"):
    """
    Create a *new* GR sequence by reassigning containers to positions.

    The set of positions remains the same; only container assignments change.

    Strategies:
        - "shuffle":
            Full random permutation of all containers.
        - "swap_pairs":
            Perform a number of random pair swaps (milder change).
        - "block_rotate":
            For each row (same first index in 'row.col'), rotate containers
            one step within that row.
        - any other value:
            Defaults to a full random shuffle.
    """
    rng = random.Random(seed)
    positions, containers = extract_positions_and_containers(gr_sequence)
    new_containers = containers[:]

    if strategy == "shuffle":
        rng.shuffle(new_containers)

    elif strategy == "swap_pairs":
        swaps = max(1, len(new_containers) // 5)
        for _ in range(swaps):
            i = rng.randrange(len(new_containers))
            j = rng.randrange(len(new_containers))
            new_containers[i], new_containers[j] = new_containers[j], new_containers[i]

    elif strategy == "block_rotate":
        # rotate within each row (1.x together, 2.x together, etc.)
        rows = {}
        for idx, pos in enumerate(positions):
            try:
                r, c = pos.split(".")
                rows.setdefault(int(r), []).append((int(c), idx))
            except Exception:
                rows.setdefault(0, []).append((idx, idx))

        out = new_containers[:]
        for r, lst in rows.items():
            lst.sort()  # by column
            idxs = [j for _, j in lst]
            if not idxs:
                continue
            first = out[idxs[0]]
            for k in range(len(idxs) - 1):
                out[idxs[k]] = out[idxs[k + 1]]
            out[idxs[-1]] = first
        new_containers = out

    else:
        # default to random shuffle for unknown strategy
        rng.shuffle(new_containers)

    return [{pos: cont} for pos, cont in zip(positions, new_containers)]

def apply_gr_sequence_to_data(data, new_gr_sequence):
    """
    Return a deep-copied data payload with an updated 'gr_sequence'.

    This function:
        - Takes an existing input `data` (e.g. optimization or simulation payload).
        - Creates a deep copy to avoid mutating the original.
        - Replaces the 'gr_sequence' entry with a normalized `new_gr_sequence`.

    """
    new_data = copy.deepcopy(data)
    new_data["gr_sequence"] = normalize_gr_sequence(new_gr_sequence)
    return new_data
