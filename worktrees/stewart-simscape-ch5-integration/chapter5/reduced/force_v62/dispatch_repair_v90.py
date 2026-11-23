"""Long predecessor wait for observation-only follow-up after a frozen batch."""
import argparse
import json
import sys
import time
import dispatch
from dispatch_repair_v89 import verify, wait


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--after-ledger', required=True)
    parser.add_argument('--after-function', required=True)
    parser.add_argument('functions', nargs='+')
    args = parser.parse_args()
    verify()
    deadline = time.monotonic() + 10800
    path = dispatch.OUT / args.after_ledger
    while True:
        try:
            entries = json.loads(path.read_text(encoding='utf-8-sig'))
        except (OSError, json.JSONDecodeError):
            entries = []
        if entries:
            if entries[-1]['result']['state'] == 'failed':
                raise RuntimeError('Previous batch failed; preserve the failure and stop.')
            if entries[-1]['function'] == args.after_function and entries[-1]['result']['state'] == 'complete':
                break
        if time.monotonic() > deadline:
            raise TimeoutError('Three-hour predecessor wait: ' + str(path))
        time.sleep(5)
    dispatch.wait = wait
    sys.argv = [sys.argv[0], '--name', args.name, *args.functions]
    dispatch.main()
    verify()
