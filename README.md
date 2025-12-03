## Description

This code is designed to optimize the robot picking sequence in a bipartite setting with a dual gripper.  
The objective is to minimize the total travel time required for the robot to:

- pick components from Gravity Rack (GR) positions, and  
- place them into the correct Kit Holder (KH) slots,

while the robot can carry up to two components at the same time.

In the real-world application, we work with a detailed mapping between containers, Gravity Rack positions and Kit Holders.  
Each KH position requires specific component types, and each GR pocket provides specific components.  
On top of this mapping we implement two methods:

- a **dual-gripper heuristic**, which is a nearest-neighbor–style algorithm extended to handle up to two components in hand (pick→pick→place→place when possible), and  
- a **dual-gripper linear baseline**, which follows a more structured, rule-based picking and placing pattern.

By comparing the results (total travel time) of the heuristic and the linear baseline on the same instances, we can study the trade-offs between solution quality and simplicity of the method in realistic MODAPTO kitting scenarios.

In addition, a **simulation mode** explores alternative GR layouts: starting from a baseline `gr_sequence`, the service shuffles container positions multiple times and evaluates how much the picking performance can be improved by rearranging the Gravity Rack.

## How to run (dual-gripper service)

The dual-gripper service is executed as a simple Python script using `main.py`.
It can run in two modes, depending on the `method` field inside the JSON input:

- `method = "heuristic"`  
- `method = "linear"`  
- `method = "heuristic-linear"`  
  → run optimization (dual-gripper heuristic / linear).

- `method = "simulation"`  
  → run simulation (random GR layouts with the dual gripper).

## How to use Optimization / Simulation (dual gripper)

- **Optimization (dual-gripper)**  
  The optimization mode uses the real GR / KH mapping and the dual-gripper logic.  
  You can select one of the following values in `data.method`:
  - `"heuristic"` – run only the dual-gripper heuristic.
  - `"linear"` – run only the dual-gripper linear baseline.
  - `"heuristic-linear"` – run both methods and keep the one with the better (lower) total cost.

- **Simulation (dual-gripper)**  
  The simulation mode (`method = "simulation"`) keeps the same GR/KH mapping but changes the GR layout.  
  Starting from a baseline `gr_sequence`, the service generates multiple randomized GR configurations (controlled by `num_random_gr_configs`) and evaluates them with the same dual-gripper logic.  
  The result reports the baseline costs and the best found GR configuration with its improvement over the baseline.

## Input JSON (high-level overview)

All use cases follow the same basic envelope:

- Top level: an object that may contain a `uuid` and a `data` field.
- The `data` field contains:
  - a `method` field that controls which mode is used, and  
  - the configuration fields (templates, GR layout, etc.).

Typical fields inside `data` for the dual-gripper service are:

- For **dual-gripper optimization** (`method = "heuristic"`, `"linear"`, `"heuristic-linear"`): 

A `templates` block with `containers_opt`, `kit_holders_opt`, `kh_sequences_opt`, `distance_matrix_opt`, plus a specific `gr_sequence` describing the current Gravity Rack layout, and the chosen `method`. 

- For **dual-gripper simulation** (`method = "simulation"`):

A `templates` block with `containers_sim`, `kit_holders_sim`, `kh_sequences_sim`, `distance_matrix_sim`,  
  together with a baseline `gr_sequence` and `num_random_gr_configs` specifying how many random GR configurations to test. The service uses these templates to build concrete instances, randomize Gravity Rack layouts and evaluate the resulting sequences.

  The service uses these inputs to:
  1. evaluate the baseline GR layout, and  
  2. generate and evaluate multiple alternative GR layouts, reporting the best one and its improvement over the baseline.

## Data and examples

Reference JSON examples and industrial input data (distance matrices, container and KH templates) for the real-world MODAPTO application can be obtained from the following Zenodo record:

[Dataset for Pick and Place Operations (Bipartite Travelling Salesman Problem - BTSP)](https://zenodo.org/records/17640037)

You can adapt these JSONs to the dual-gripper format described above and use them (via local execution) to reproduce dual-gripper optimization and simulation scenarios.


## Authors
The persons who contributed to this project are Konstantinos Giannakos, Dimitrios Tsakoumis, Gregory Koronakos, Stathis Plitsos and Pavlos Eirinakis.

## Acknowledgement
This research work has been conducted within the framework of the MODAPTO project (MODULAR MANUFACTURING AND DISTRIBUTED CONTROL VIA INTEROPERABLE DIGITAL TWINS), which has received funding from the European Union’s Horizon Europe research and innovation program under grant agreement No 101091996.
<https://modapto.eu/>

## License
robot-picking-seq-opt (c) by the University of Piraeus, Greece.

robot-picking-seq-opt is licensed under a
Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.

You should have received a copy of the license along with this
work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.
