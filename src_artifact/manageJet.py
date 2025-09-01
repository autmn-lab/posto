#!/usr/bin/env python3
"""
Scripts to generate logs and check safety

Usage:
    manageJet.py (generateLog) [--log=<logfile>] [--init=<initialState>] [--timestamp=<timestamp>]
    manageJet.py (checkSafety) [--log=<logfile>] [--timestamp=<timestamp>] [--unsafe=<unsafe>] [--state=<state>] [--op=<op>]
"""

from docopt import docopt
import ast, os, sys, time
from Jet import *

# ---------- helpers: colored logging ----------
def info(s):    print(f"{msg.OKCYAN}[INFO]{msg.ENDC} {s}")
def note(s):    print(f"{msg.OKBLUE}[INFO]{msg.ENDC} {s}")
def ok(s):      print(f"{msg.OKGREEN}[SUCCESS]{msg.ENDC} {s}")
def warn(s):    print(f"{msg.WARNING}[WARN]{msg.ENDC} {s}")
def die(s, hint=None, code=2):
    print(f"{msg.FAIL}[ERROR]{msg.ENDC} {s}")
    if hint:
        print(f"{msg.WARNING}[HINT]{msg.ENDC} {hint}")
    sys.exit(code)

# ---------- parsing & validation ----------
def parse_initset(init_str):
    """
    Accepts formats like:
      "[0.8,1],[0.8,1]"
      " [ 0.8 , 1.0 ] , [ 0.8 , 1.0 ] "
    Returns: ([x1,x2], [y1,y2]) as floats
    """
    if init_str is None:
        die("Missing --init.",
            hint='Provide it in format: "[0.8,1],[0.8,1]"')

    try:
        parts = init_str.split("],")
        if len(parts) != 2:
            raise ValueError("Expected two bracketed ranges separated by a comma.")
        set1 = ast.literal_eval(parts[0].strip() + "]")
        set2 = ast.literal_eval(parts[1].strip())
        if (not isinstance(set1, (list, tuple)) or len(set1) != 2 or
            not isinstance(set2, (list, tuple)) or len(set2) != 2):
            raise ValueError("Each set must have exactly two numbers.")
        set1 = [float(set1[0]), float(set1[1])]
        set2 = [float(set2[0]), float(set2[1])]
        return (set1, set2)
    except Exception as e:
        die(
            f"Invalid --init format: {init_str!r}.",
            hint='Use like: "[0.8,1],[0.8,1]" (two bracketed pairs separated by a comma).'
        )

def require_path(path_str, flag_name="--log"):
    if path_str is None or str(path_str).strip() == "":
        die(f"Missing {flag_name}.",
            hint=f"Provide a valid path via {flag_name}=<file>.")
    # Ensure parent directory exists (if one is specified)
    parent = os.path.dirname(os.path.abspath(path_str))
    if parent and not os.path.isdir(parent):
        die(f"Directory does not exist for {flag_name}: {parent!r}.",
            hint="Create the directory or change the path.")
    return path_str

def require_int(val_str, flag_name, min_value=None, max_value=None):
    if val_str is None:
        die(f"Missing {flag_name}.",
            hint=f"Provide an integer via {flag_name}=<int>.")
    try:
        v = int(val_str)
    except Exception:
        die(f"{flag_name} must be an integer (got {val_str!r}).")
    if min_value is not None and v < min_value:
        die(f"{flag_name} must be >= {min_value} (got {v}).")
    if max_value is not None and v > max_value:
        die(f"{flag_name} must be <= {max_value} (got {v}).")
    return v

def require_float(val_str, flag_name, min_value=None, max_value=None):
    if val_str is None:
        die(f"Missing {flag_name}.",
            hint=f"Provide a number via {flag_name}=<float>.")
    try:
        v = float(val_str)
    except Exception:
        die(f"{flag_name} must be a number (got {val_str!r}).")
    if min_value is not None and v < min_value:
        die(f"{flag_name} must be >= {min_value} (got {v}).")
    if max_value is not None and v > max_value:
        die(f"{flag_name} must be <= {max_value} (got {v}).")
    return v

def require_op(op_str):
    """
    Accept symbols or words; normalize to the word form used by TrajSafety:
      lt (<), le (<=), gt (>), ge (>=), eq (==), ne (!=)
    """
    if op_str is None:
        die("Missing --op.", hint="Use one of: <, <=, >, >=, ==, !=, lt, le, gt, ge, eq, ne")

    s = op_str.strip().lower()
    alias = {
        "<": "lt", "<=": "le", ">": "gt", ">=": "ge", "==": "eq", "!=": "ne",
        "lt": "lt", "le": "le", "gt": "gt", "ge": "ge", "eq": "eq", "ne": "ne",
    }
    if s not in alias:
        die(f"Invalid --op: {op_str!r}.",
            hint="Allowed: <, <=, >, >=, ==, != or lt, le, gt, ge, eq, ne")
    return alias[s]



# ---------- main ----------
if __name__ == '__main__':
    args = docopt(__doc__)

    if args['generateLog']:
        log = require_path(args['--log'], "--log")
        init = parse_initset(args['--init'])
        timestamp = require_int(args['--timestamp'], "--timestamp", min_value=0)

        info("Starting log generation...")
        note(f"Initial set: {init}")
        note(f"Time horizon: {timestamp}")
        note(f"Output log path: {log}")

        start = time.time()
        try:
            Jet.generateLog(init, timestamp, log)
        except Exception as e:
            die(f"Log generation failed: {e!r}",
                hint="Check your inputs and file permissions.")
        elapsed = time.time() - start

        ok("Log generated successfully.")
        ok(f"Stored at: {msg.UNDERLINE}{log}{msg.ENDC}")
        print(f"{msg.HEADER}[INFO]{msg.ENDC} Time taken: {msg.BOLD}{elapsed:.4f} sec{msg.ENDC}")

    elif args['checkSafety']:
        log = require_path(args['--log'], "--log")
        timestamp = require_int(args['--timestamp'], "--timestamp", min_value=0)
        unsafe = require_float(args['--unsafe'], "--unsafe")
        state = require_int(args['--state'], "--state", min_value=0)
        op = require_op(args['--op'])

        info("Running safety check...")
        note(f"Log path: {log}")
        note(f"Time horizon: {timestamp}")
        note(f"Unsafe threshold: {unsafe}")
        note(f"State index: {state}")
        note(f"Operator: {op}")

        try:
            Jet.checkSafety(log, timestamp, unsafe, state, op)
            ok("Safety check completed.")
        except Exception as e:
            die(f"Safety check failed: {e!r}",
                hint="Verify the log file exists and input parameters are correct.")

    else:
        warn("No command provided. Use 'generateLog' or 'checkSafety'.")
        print(__doc__)
        sys.exit(1)
