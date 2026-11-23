"""Run the frozen repair batch in the existing worker with an explicit budget."""
import hashlib
import json
import time
import dispatch


def wait(job=None):
    start = notice = time.monotonic()
    while True:
        value = dispatch.state()
        if value.get('state') in ('complete', 'failed') and (job is None or value.get('id') == job):
            return value
        if time.monotonic() - notice > 60:
            print('Waiting for repair batch', job, flush=True)
            notice = time.monotonic()
        if time.monotonic() - start > 10800:
            raise TimeoutError('Three-hour development batch limit: ' + str(job))
        time.sleep(2)


def verify():
    identity = json.loads((dispatch.OUT / 'teacher_repair_v89/identity.json').read_text(encoding='utf-8-sig'))
    for name, expected in {**identity['original_sources'], **identity['sources']}.items():
        path = dispatch.ROOT / name
        assert path.exists() and hashlib.sha256(path.read_bytes()).hexdigest() == expected, name


if __name__ == '__main__':
    verify()
    dispatch.wait = wait
    dispatch.main()
    verify()
