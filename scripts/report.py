#!/usr/bin/env python3
"""Print a Markdown summary table from scenario result files.

Usage:
    python3 scripts/report.py results/*.json
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from metrics import format_table  # noqa: E402


def main():
    results = [json.loads(Path(p).read_text()) for p in sys.argv[1:] if Path(p).exists()]
    if not results:
        print("No scenario results found.")
        sys.exit(1)
    print(format_table(results))


if __name__ == "__main__":
    main()
