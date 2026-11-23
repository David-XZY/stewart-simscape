"""Attach this helper and its future children to a Windows lifetime job.

Keep the non-inheritable handle for this process's entire lifetime. Windows
closes it on normal or forced exit and terminates remaining compiler children.
No CPU, memory, or time limit is imposed. See Microsoft Job Objects docs.
"""
import ctypes as C
from ctypes import wintypes as W
import os

_active_job = None


class Basic(C.Structure):
    _fields_ = [('process_time', C.c_longlong), ('job_time', C.c_longlong), ('flags', W.DWORD),
                ('minimum_working_set', C.c_size_t), ('maximum_working_set', C.c_size_t),
                ('active_processes', W.DWORD), ('affinity', C.c_size_t),
                ('priority', W.DWORD), ('scheduling', W.DWORD)]


class IO(C.Structure):
    _fields_ = [(name, C.c_ulonglong) for name in
                ['read_ops', 'write_ops', 'other_ops', 'read_bytes', 'write_bytes', 'other_bytes']]


class Extended(C.Structure):
    _fields_ = [('basic', Basic), ('io', IO), ('process_memory', C.c_size_t),
                ('job_memory', C.c_size_t), ('peak_process_memory', C.c_size_t),
                ('peak_job_memory', C.c_size_t)]


def attach_current_process():
    global _active_job
    assert os.name == 'nt'
    if _active_job is not None:
        return _active_job
    kernel = C.WinDLL('kernel32', use_last_error=True)
    kernel.CreateJobObjectW.argtypes = [C.c_void_p, W.LPCWSTR]
    kernel.CreateJobObjectW.restype = W.HANDLE
    kernel.GetCurrentProcess.restype = W.HANDLE
    kernel.SetInformationJobObject.argtypes = [W.HANDLE, C.c_int, C.c_void_p, W.DWORD]
    kernel.AssignProcessToJobObject.argtypes = [W.HANDLE, W.HANDLE]
    kernel.CloseHandle.argtypes = [W.HANDLE]
    job = kernel.CreateJobObjectW(None, None)
    if not job:
        raise C.WinError(C.get_last_error())
    limits = Extended();limits.basic.flags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
    if not kernel.SetInformationJobObject(job, 9, C.byref(limits), C.sizeof(limits)):
        error = C.get_last_error();kernel.CloseHandle(job);raise C.WinError(error)
    if not kernel.AssignProcessToJobObject(job, kernel.GetCurrentProcess()):
        error = C.get_last_error();kernel.CloseHandle(job);raise C.WinError(error)
    _active_job = job  # Intentionally never close while this process is alive.
    return job
