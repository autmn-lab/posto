#!/usr/bin/env python3
"""
Scripts to generate logs and check safety

Usage:
    manageJet.py (generateLog) [--log=<logfile>] [--init=<initialState>] [--timestamp=<timestamp>]
    manageJet.py (checkSafety) [--log=<logfile>] [--timestamp=<timestamp>] [--unsafe=<unsafe>] [--state=<state>] [--op=<op>]
"""

from docopt import docopt
import ast
from Jet import *

""" def parse_init(s):

    if s is None:
        raise ValueError("--init is required (e.g., --init='0.9,0.9')")
    try:
        # If it's already bracketed/parenthesized, literal_eval it
        txt = s.strip()
        if (txt.startswith("(") and txt.endswith(")")) or (txt.startswith("[") and txt.endswith("]")):
            val = ast.literal_eval(txt)
            if len(val) != 2:
                raise ValueError
            return float(val[0]), float(val[1])
        # Otherwise accept "a,b"
        parts = [p.strip() for p in txt.split(",")]
        if len(parts) != 2:
            raise ValueError
        return float(parts[0]), float(parts[1])
    except Exception:
        raise ValueError("Invalid --init. Use formats like '0.9,0.9' or '(0.9, 0.9)'.")
 """


def parse_initset(init_str):
    if init_str is None:
        return ([0.8, 1.0], [0.8, 1.0])  # sensible default

    try:
        parts = init_str.split("],")
        set1 = ast.literal_eval(parts[0] + "]")
        set2 = ast.literal_eval(parts[1])

        # coerce to floats and validate 2 numbers each
        set1 = [float(set1[0]), float(set1[1])]
        set2 = [float(set2[0]), float(set2[1])]
        if len(set1) != 2 or len(set2) != 2:
            raise ValueError
        return (set1, set2)
    except Exception as e:
        raise ValueError(f'Invalid --initSet format. Use like: "[0.8,1],[0.8,1]". Error: {e}')



if __name__ == '__main__':
    args = docopt(__doc__)

    if args['generateLog']:
        log = args['--log']
        init = parse_initset(args['--init'])
        time = int(args['--timestamp'])
        Jet.generateLog(init, time, log)

    elif args['checkSafety']:
        log = args['--log']
        time = int(args['--timestamp'])
        unsafe = float(args['--unsafe'])
        state = int(args['--state'])
        op = args['--op']
        Jet.checkSafety(log, time, unsafe, state, op)