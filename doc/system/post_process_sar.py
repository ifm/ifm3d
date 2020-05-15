#!/usr/bin/env python

#
# Brute-force script to clean up my sar output for purposes of slurping into
# pandas for analysis/visualization.
#

import datetime
import time
import fileinput
import sys

def main():
    got_first_header = False

    for line in fileinput.input():
        # kill the sar header line
        if line.startswith("Linux"):
            continue

        # get the components (columns) of the line
        cols = line.split()

        # kill empty lines
        if len(cols) == 0:
            continue

        # only print column headers once
        if ((cols[2] == 'CPU') or (cols[2] == 'IFACE')):
            if got_first_header:
                continue
            else:
                got_first_header = True

        # filter out the 'all' cpu stats
        if (cols[2] == 'all'):
            continue

        # convert dates into datetime format
        # hack: we assume "today"
        now = datetime.datetime.now()
        t_str = ":".join(["%04d" % now.year,
                          "%02d" % now.month,
                          "%02d" % now.day]) + \
                ":" + cols[0] + ' ' + cols[1]
        t_struct = time.strptime(t_str, "%Y:%m:%d:%I:%M:%S %p")
        dt = datetime.datetime.fromtimestamp(time.mktime(t_struct))

        # get the data we care about
        chopped_cols = cols[2:]

        # insert our time or time header
        if ((cols[2] == 'CPU') or (cols[2] == 'IFACE')):
            out_cols = ['Time'] + chopped_cols
        else:
            out_cols = [dt] + chopped_cols

        # print as a csv
        print(",".join(map(str, out_cols)))

    return 0

if __name__  == '__main__':
    sys.exit(main())
