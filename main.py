# REPOSITORY NAME (c) by the University of Piraues, Greece.
#
# REPOSITORY NAME is licensed under a
# Creative Commons Attribution-NonCommercial-NoDerivs 3.0 Unported License.
#
# You should have received a copy of the license along with this
# work.  If not, see <http://creativecommons.org/licenses/by-nc-nd/3.0/>.

import json
import sys,traceback
from datetime import datetime
from time import time
import pika

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

# ───────────────────────── RabbitMQ callback ─────────────────────────
# RabbitMQ callback to process incoming jobs.
#
# Steps:
#     1. Parse the incoming JSON message.
#     2. Execute solution method, i.e., simulation or optimization.
#     3. Publish encoded results back to RabbitMQ (`opt-result` exchange).
#
# On error:
#     - Logs the traceback.
#     - Sends an error response back to RabbitMQ.
def callback(ch, method, properties, body):
    input_file = json.loads(body)
    uuid = input_file.get('uuid', 'unknown')
    try:
        print(f"{datetime.now():%d/%m/%Y %H:%M:%S}: Job {uuid} received.")
        data = input_file["data"]
        print(data)

        print("1. Solving instance.")
        result = run_from_msg(data)

        output = {
            "uuid": uuid,
            "produced_at": int(time() * 1000),
            "data": result
        }
        print("2. Rabbit publishing.")
        ch.basic_publish(exchange='opt-result',
                         routing_key='dual-gripper-robot-picking-seq',
                         body=json.dumps(output))
        print(f"{datetime.now():%d/%m/%Y %H:%M:%S}: Job {uuid} completed.")
    except Exception as exc:
        traceback.print_exc()
        error_responce = {
            "message": f"Problem in input data: {exc}"
        }
        print("2. EXCEPTION: Rabbit publishing.")
        ch.basic_publish(exchange='opt-result',
                         routing_key='dual-gripper-robot-picking-seq',
                         body=json.dumps(error_responce))
        print(f"{datetime.now():%d/%m/%Y %H:%M:%S}: Job {uuid} failed.")



# Command-line entry point for dual-gripper optimization/simulation.
#
# Behavior:
#     - If REMOTE mode is on, then the service consumes a JSON file as submitted in the subscribed queue via the
#       `run_from_msg` function.
#     - If REMOTE mode is off:
#       - If no JSON path is provided via command-line, uses DEFAULT_JSON.
#       - Loads the JSON file into a Python dict.
#       - Passes the message to `run_from_msg` for processing.
#
# Command-line usage:
#     1. python main.py [REMOTE RABBIT_HOST RABBIT_PORT RABBIT_USER RABIT_PASSWORD]
#     2. python main.py [REMOTE optional_input_path]
# Where:
#     REMOTE (int, 0 or 1):
#         Equals to 1 if a remote connection to a rabbit-mq queue is preferable.
#         Equals to 0 if local mode is preferable
#     RABBIT_HOST (str, mandatory if REMOTE==1):
#         Connection URL to the preferable rabbit-mq server.
#     RABBIT_PORT (str, mandatory if REMOTE==1):
#         Connection port to the preferable rabbit-mq server.
#     RABBIT_USER (str, mandatory if REMOTE==1):
#         Username used for authentication from the preferable rabbit-mq server.
#     RABIT_PASSWORD (str, mandatory if REMOTE==1):
#         Password used for authentication from the preferable rabbit-mq server.
#     optional_input_path (str, optional):
#         Path to a JSON file containing the "data" block and `method`.
#         If omitted, DEFAULT_JSON is used.
def main():
    online = sys.argv[1]

    if online == "1":
        host, port, user, pw = sys.argv[2:7]
        conn = pika.BlockingConnection(pika.ConnectionParameters(
            host, int(port), '/', pika.PlainCredentials(user, pw),
            heartbeat=1800, blocked_connection_timeout=900))
        channel = conn.channel()
        channel.basic_consume(queue='dual-gripper-robot-picking-seq_job',
                              auto_ack=True,
                              on_message_callback=callback)
        print(" [*] Waiting for messages.")
        channel.start_consuming()

    elif online == "0":
        if len(sys.argv) < 3:
            json_path = DEFAULT_JSON
            print(f"(no arg given) → using default JSON: {json_path}")
        else:
            json_path = sys.argv[1]

        with open(json_path, "r") as f:
            msg = json.load(f)

        run_from_msg(msg)


if __name__ == "__main__":
    main()
