#!/usr/bin/env python3
"""Compare a candidate observer's event JSONL against the actual source golden."""
import argparse
import json
import math
from pathlib import Path


def compare(expected, actual, path, atol, rtol):
    if isinstance(expected, bool) or isinstance(expected, (str, int)) or expected is None:
        if type(expected) is not type(actual) or expected != actual:
            raise AssertionError(f'{path}: expected {expected!r}, got {actual!r}')
    elif isinstance(expected, float):
        if not isinstance(actual, (int, float)) or not math.isfinite(actual):
            raise AssertionError(f'{path}: non-finite/non-numeric {actual!r}')
        if abs(expected-actual) > atol + rtol*abs(expected):
            raise AssertionError(f'{path}: expected {expected:.17g}, got {actual:.17g}')
    elif isinstance(expected, list):
        if not isinstance(actual, list) or len(expected) != len(actual):
            raise AssertionError(f'{path}: shape/length mismatch')
        for index, (a, b) in enumerate(zip(expected, actual)):
            compare(a, b, f'{path}[{index}]', atol, rtol)
    elif isinstance(expected, dict):
        if not isinstance(actual, dict) or set(expected) != set(actual):
            raise AssertionError(f'{path}: keys mismatch')
        for key in expected:
            compare(expected[key], actual[key], f'{path}.{key}', atol, rtol)
    else:
        raise TypeError(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('actual', type=Path)
    parser.add_argument('--expected', type=Path,
                        default=Path(__file__).with_name('expected_outputs.jsonl'))
    parser.add_argument('--atol', type=float, default=1e-8)
    parser.add_argument('--rtol', type=float, default=1e-10)
    args = parser.parse_args()
    expected = [json.loads(x) for x in args.expected.read_text().splitlines()]
    actual = [json.loads(x) for x in args.actual.read_text().splitlines()]
    compare(expected, actual, 'events', args.atol, args.rtol)
    print(json.dumps(dict(pass_=True, events=len(expected), atol=args.atol, rtol=args.rtol)))


if __name__ == '__main__':
    main()
