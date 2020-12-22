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

def run_lsd(infile, outpath):
    fname, ext = os.path.basename(infile).split(".")
    subprocess.run([
        "./lsd",
        infile,
        f"{outpath}/lines_{fname}.txt"
    ])

assert len(sys.argv) == 3

infilepattern = sys.argv[1]
outpath = sys.argv[2]

infiles = list(glob.glob(infilepattern))

for i, infile in enumerate(infiles):
    run_lsd(infile, outpath)
    progress_bar(i, len(infiles))
