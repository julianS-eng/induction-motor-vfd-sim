"""Inject the generated result tables (docs/results/*.md) into README.md.

Every number shown in the README comes from the experiment scripts: they write
``docs/results/<name>.md`` and this script copies those files between the
markers ``<!-- BEGIN results/<name>.md -->`` and ``<!-- END results/<name>.md -->``.

Usage::

    python scripts/sync_readme.py          # rewrite README.md in place
    python scripts/sync_readme.py --check  # fail if README.md is out of date (CI)
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATTERN = re.compile(
    r"(<!-- BEGIN results/(?P<name>[\w.-]+) -->\n)(?P<body>.*?)(<!-- END results/(?P=name) -->)",
    re.DOTALL,
)


def render(readme: str, results_dir: Path) -> str:
    """Return ``readme`` with every marker block replaced by its results file."""

    def repl(match: re.Match[str]) -> str:
        path = results_dir / match.group("name")
        if not path.exists():
            raise FileNotFoundError(f"{path} referenced by README.md does not exist")
        body = path.read_text(encoding="utf-8").strip() + "\n"
        return match.group(1) + body + match.group(4)

    return PATTERN.sub(repl, readme)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="only verify, do not write")
    args = parser.parse_args()
    readme_path = ROOT / "README.md"
    current = readme_path.read_text(encoding="utf-8")
    updated = render(current, ROOT / "docs" / "results")
    if args.check:
        if updated != current:
            print("README.md is out of sync with docs/results; run scripts/sync_readme.py")
            return 1
        print("README.md is in sync with docs/results")
        return 0
    readme_path.write_text(updated, encoding="utf-8")
    print("README.md updated")
    return 0


if __name__ == "__main__":
    sys.exit(main())
