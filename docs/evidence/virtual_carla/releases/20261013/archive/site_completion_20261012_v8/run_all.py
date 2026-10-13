#!/usr/bin/env python3
"""HH_261012 - Same all-13 supervisor, UI probe, capture, and ownership protections."""
from _reuse import load

runner = load("run_all")

if __name__ == "__main__":
    raise SystemExit(runner.main())


