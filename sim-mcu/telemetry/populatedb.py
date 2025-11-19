# pip install paho-mqtt
import json, time, random
import paho.mqtt.client as mqtt

c = mqtt.Client()
c.connect("localhost", 1883, 60)

while True:
    ts = int(time.time()*1000)
    att = dict(ts=ts, fw="0.7.3", mode="PD",
               q_w=1.0, q_x=random.uniform(-0.05,0.05),
               q_y=random.uniform(-0.05,0.05), q_z=random.uniform(-0.05,0.05),
               w_x=random.uniform(-0.02,0.02), w_y=random.uniform(-0.02,0.02), w_z=random.uniform(-0.02,0.02))
    rw = dict(ts=ts,
              rw1=random.uniform(80,150), rw2=random.uniform(80,150), rw3=random.uniform(80,150),
              rw1_sat=False, rw2_sat=False, rw3_sat=False)
    c.publish("sat/esp32s3/sat-01/attitude", json.dumps(att), qos=1)
    c.publish("sat/esp32s3/sat-01/rwheels", json.dumps(rw), qos=1)
    time.sleep(1)

