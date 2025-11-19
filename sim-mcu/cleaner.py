## script para limpiar datos crudos generados por RTT
import re

infile = "mcu_raw_rtt.txt"
outfile = "sim_output.csv"

# Match header or numeric CSV lines like:
# "16:59:45.007: time,..."  or  "16:59:45.007: 0.020000,..."
prefix_strip = re.compile(r'^\d{2}:\d{2}:\d{2}\.\d{3}:\s*')  # remove "hh:mm:ss.mmm:"
csv_line = re.compile(r'^[\d\-\+\.eE]+,')                     # detect data lines after strip

header_seen = False

with open(infile, "r", errors="ignore") as inf, open(outfile, "w") as outf:
    for raw in inf:
        line = raw.strip()
        if not line:
            continue

        # Remove timestamp prefix if present
        line = prefix_strip.sub("", line)

        # Handle header line
        if not header_seen and line.startswith("time,"):
            outf.write(line + "\n")
            header_seen = True
            continue

        # Handle numeric data rows
        if header_seen and csv_line.match(line):
            outf.write(line + "\n")

print(f"[+] Clean CSV written to {outfile}")

