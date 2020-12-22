import subprocess
import glob
import sys
import os

def progress_bar(current, total):
    i = current + 1
    bar_fill = '=' * (i * 50 // total)
    end_of_line = "\n" if (i == total) else ""
    sys.stdout.write("\r[%-50s] %d/%d%s" % (bar_fill, i, total, end_of_line))
    sys.stdout.flush()

def run_vp(infile):
    fname = os.path.basename(infile)
    timestamp = float(fname.split(".")[0].split("_")[-1]) * 0.001
    proc = subprocess.run(
        ["./vp", infile],
        capture_output=True
    )
    errs = proc.stderr.decode()
    if errs:
        sys.stderr.write(errs)
        exit()
    print(f"{timestamp} {proc.stdout.decode().strip()}")

assert len(sys.argv) == 2

infilepattern = sys.argv[1]

infiles = list(glob.glob(infilepattern))

for i, infile in enumerate(infiles):
    run_vp(infile)
    # progress_bar(i, len(infiles))
