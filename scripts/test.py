#!/usr/bin/env python3
"""Run the ODM unit test suite.

Cross-platform replacement for the old test.sh/test.bat pair. Invoked via
`pixi run test` (optionally `pixi run test -- <name>` to run a single
test_<name>.py module).
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    # Tests import the `opendm` package and reference assets via repo-relative
    # paths (e.g. "tests/assets/..."), so run from the repo root.
    os.chdir(ROOT)
    sys.path.insert(0, ROOT)

    name = sys.argv[1] if len(sys.argv) > 1 else "*"
    suite = unittest.TestLoader().discover("tests", pattern="test_%s.py" % name)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
