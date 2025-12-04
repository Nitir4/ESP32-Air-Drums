"""Convenience runner: ensures project root is on sys.path and runs src.main.

Use this if `python -m src.main` fails due to module path issues in some environments.
Run from project root:

    # Default (scans for "ESPDRUMS*" BLE devices):
    python run.py
    
    # Custom BLE device name prefix:
    python run.py --device-prefix "MYDRUMS"

"""
from __future__ import annotations

import os
import sys
from typing import List


def main(argv: List[str] | None = None) -> None:
    # Ensure project root (directory containing this file) is on sys.path
    project_root = os.path.dirname(os.path.abspath(__file__))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    # Import and call the package entry point
    from src.main import main as src_main

    src_main(argv)


if __name__ == "__main__":
    import sys as _sys

    main(_sys.argv[1:])
