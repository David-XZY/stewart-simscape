"""Read Windows process-tree private bytes and machine commit without dependencies."""
import argparse
import ctypes as C
from ctypes import wintypes as W
import json
import os
from pathlib import Path
import time


class Entry(C.Structure):
    _fields_ = [('dwSize', W.DWORD), ('cntUsage', W.DWORD), ('pid', W.DWORD),
                ('heap', C.c_size_t), ('module', W.DWORD), ('threads', W.DWORD),
                ('parent', W.DWORD), ('priority', W.LONG), ('flags', W.DWORD), ('name', W.WCHAR * 260)]


class Counters(C.Structure):
    _fields_ = [('cb', W.DWORD), ('faults', W.DWORD), *[(n, C.c_size_t) for n in
                ['peak_rss', 'rss', 'peak_paged_pool', 'paged_pool', 'peak_nonpaged_pool',
                 'nonpaged_pool', 'pagefile', 'peak_pagefile', 'private']]]


class Machine(C.Structure):
    _fields_ = [('length', W.DWORD), ('load', W.DWORD), *[(n, C.c_ulonglong) for n in
                ['total_physical', 'available_physical', 'commit_limit', 'available_commit',
                 'total_virtual', 'available_virtual', 'available_extended']]]


def snapshot(pid):
    assert os.name == 'nt'
    kernel = C.WinDLL('kernel32', use_last_error=True)
    psapi = C.WinDLL('psapi', use_last_error=True)
    kernel.CreateToolhelp32Snapshot.argtypes = [W.DWORD, W.DWORD]
    kernel.CreateToolhelp32Snapshot.restype = W.HANDLE
    kernel.Process32FirstW.argtypes = kernel.Process32NextW.argtypes = [W.HANDLE, C.POINTER(Entry)]
    kernel.OpenProcess.argtypes = [W.DWORD, W.BOOL, W.DWORD]
    kernel.OpenProcess.restype = W.HANDLE
    kernel.CloseHandle.argtypes = [W.HANDLE]
    psapi.GetProcessMemoryInfo.argtypes = [W.HANDLE, C.POINTER(Counters), W.DWORD]
    kernel.GlobalMemoryStatusEx.argtypes = [C.POINTER(Machine)]
    handle = kernel.CreateToolhelp32Snapshot(2, 0)
    entry = Entry();entry.dwSize = C.sizeof(entry)
    parents = {};names = {}
    try:
        more = kernel.Process32FirstW(handle, C.byref(entry))
        while more:
            parents[entry.pid] = entry.parent;names[entry.pid] = entry.name
            more = kernel.Process32NextW(handle, C.byref(entry))
    finally:
        kernel.CloseHandle(handle)
    ids = {pid}
    while True:
        expanded = ids | {p for p, parent in parents.items() if parent in ids}
        if expanded == ids:break
        ids = expanded
    processes = []
    for child in sorted(ids & parents.keys()):
        handle = kernel.OpenProcess(0x0400 | 0x0010, False, child)
        if not handle:continue
        counters = Counters();counters.cb = C.sizeof(counters)
        try:
            if psapi.GetProcessMemoryInfo(handle, C.byref(counters), counters.cb):
                processes.append({'pid': child, 'name': names[child], 'private_bytes': counters.private,
                                  'resident_bytes': counters.rss, 'peak_resident_bytes': counters.peak_rss})
        finally:
            kernel.CloseHandle(handle)
    machine = Machine();machine.length = C.sizeof(machine)
    assert kernel.GlobalMemoryStatusEx(C.byref(machine))
    return {'utc_seconds': time.time(), 'root_pid': pid, 'running': pid in parents,
            'processes': processes, 'private_bytes': sum(p['private_bytes'] for p in processes),
            'available_physical_bytes': machine.available_physical,
            'available_commit_bytes': machine.available_commit, 'commit_limit_bytes': machine.commit_limit}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--interval', type=float, default=5)
    args = parser.parse_args()
    with args.output.open('a', encoding='utf-8') as handle:
        while True:
            sample = snapshot(args.pid);handle.write(json.dumps(sample) + '\n');handle.flush()
            if not sample['running']:break
            time.sleep(args.interval)


if __name__ == '__main__':main()
