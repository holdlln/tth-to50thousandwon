"""Run core tests and preserve actual results for the validation report."""
import io
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    suite = unittest.defaultTestLoader.discover(str(ROOT / 'tests'))
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    output = ROOT / 'output/validation'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'tests.log').write_text(stream.getvalue(), encoding='utf-8')
    evidence = dict(command='python tools/run_tests.py', count=result.testsRun,
                    result='OK' if result.wasSuccessful() else 'FAILED',
                    failures=len(result.failures), errors=len(result.errors))
    (output / 'tests.json').write_text(json.dumps(evidence, indent=2), encoding='utf-8')
    print(stream.getvalue())
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
