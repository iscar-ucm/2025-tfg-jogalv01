while true; do
  TS_MS=$(($(date +%s%3N)))
  mosquitto_pub -h localhost -p 1883 -t sat/esp32s3/sat-01/attitude \
    -m "{\"ts\":$TS_MS,\"fw\":\"0.7.3\",\"mode\":\"PD\",\"q_w\":1,\"q_x\":0.01,\"q_y\":-0.01,\"q_z\":0.02,\"w_x\":0.01,\"w_y\":0.02,\"w_z\":0.03}"
  mosquitto_pub -h localhost -p 1883 -t sat/esp32s3/sat-01/rwheels \
    -m "{\"ts\":$TS_MS,\"rw1\":$((RANDOM%200)) ,\"rw2\":$((RANDOM%200)) ,\"rw3\":$((RANDOM%200)), \"rw1_sat\":false,\"rw2_sat\":false,\"rw3_sat\":false}"
  sleep 1
done

