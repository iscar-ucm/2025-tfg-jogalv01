import csv
import json
import time
from paho.mqtt import client as mqtt

BROKER = "localhost"
PORT = 1883
TOPIC = "sat/esp32s3/sat-01/attitude"

CSV_FILE = "telemetry.csv"

def mqtt_connect():
    client = mqtt.Client()
    client.connect(BROKER, PORT, keepalive=60)
    return client


def publish_csv_rows(client):
    with open(CSV_FILE, "r") as f:
        reader = csv.DictReader(f)

        for row in reader:
            # Convert CSV strings → floats
            msg = {
                "ts": int(time.time() * 1000),
                "fw": "0.7.3",
                "mode": "PD",

                "q_w": float(row["q_w"]),
                "q_x": float(row["q_x"]),
                "q_y": float(row["q_y"]),
                "q_z": float(row["q_z"]),

                "w_x": float(row["w_x"]),
                "w_y": float(row["w_y"]),
                "w_z": float(row["w_z"]),

                "rw1": float(row["rw1"]),
                "rw2": float(row["rw2"]),
                "rw3": float(row["rw3"]),

                # basic saturation flags (example)
                "rw1_sat": abs(float(row["rw1"])) >= 3500,
                "rw2_sat": abs(float(row["rw2"])) >= 3500,
                "rw3_sat": abs(float(row["rw3"])) >= 3500,
            }

            payload = json.dumps(msg)
            client.publish(TOPIC, payload)

            # Optional: throttle publish speed
            time.sleep(0.02)   # 20 ms (matches your dt=0.02)


def main():
    client = mqtt_connect()
    publish_csv_rows(client)
    print("Done!")


if __name__ == "__main__":
    main()

