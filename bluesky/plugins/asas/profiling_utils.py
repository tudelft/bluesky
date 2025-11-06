"""
Performance profiling utilities for BlueSky DAA/SSD plugins.

Provides:
- Function-level timing decorators
- Cumulative statistics tracking
- Memory profiling
- Hotspot detection
- Profile report generation
"""

import time
import functools
import cProfile
import pstats
import io
from collections import defaultdict
from typing import Dict, List, Callable, Any
import numpy as np

# Global profiling state
_profiling_enabled = False
_profile_stats: Dict[str, Dict[str, Any]] = defaultdict(lambda: {
    'count': 0,
    'total_time': 0.0,
    'min_time': float('inf'),
    'max_time': 0.0,
    'times': []
})

_cprofile_enabled = False
_cprofile_profiler = None


def enable_profiling():
    """Enable function-level profiling."""
    global _profiling_enabled
    _profiling_enabled = True


def disable_profiling():
    """Disable function-level profiling."""
    global _profiling_enabled
    _profiling_enabled = False


def enable_cprofile():
    """Enable cProfile detailed profiling."""
    global _cprofile_enabled, _cprofile_profiler
    _cprofile_enabled = True
    _cprofile_profiler = cProfile.Profile()


def disable_cprofile():
    """Disable cProfile profiling."""
    global _cprofile_enabled, _cprofile_profiler
    _cprofile_enabled = False
    if _cprofile_profiler:
        _cprofile_profiler = None


def is_profiling_enabled():
    """Check if profiling is enabled."""
    return _profiling_enabled


def is_cprofile_enabled():
    """Check if cProfile is enabled."""
    return _cprofile_enabled


def profile_function(name: str = None):
    """
    Decorator to profile function execution time.
    
    Args:
        name: Optional custom name for the function in reports
    
    Usage:
        @profile_function()
        def my_function():
            pass
        
        @profile_function("Custom Name")
        def another_function():
            pass
    """
    def decorator(func: Callable) -> Callable:
        func_name = name or f"{func.__module__}.{func.__name__}"
        
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            if not _profiling_enabled:
                return func(*args, **kwargs)
            
            start_time = time.perf_counter()
            try:
                result = func(*args, **kwargs)
                return result
            finally:
                elapsed = time.perf_counter() - start_time
                
                stats = _profile_stats[func_name]
                stats['count'] += 1
                stats['total_time'] += elapsed
                stats['min_time'] = min(stats['min_time'], elapsed)
                stats['max_time'] = max(stats['max_time'], elapsed)
                
                # Keep last 1000 samples for percentile calculations
                stats['times'].append(elapsed)
                if len(stats['times']) > 1000:
                    stats['times'].pop(0)
        
        return wrapper
    return decorator


def profile_section(section_name: str):
    """
    Context manager to profile a code section.
    
    Usage:
        with profile_section("my_section"):
            # code to profile
            pass
    """
    class ProfileSection:
        def __enter__(self):
            if _profiling_enabled:
                self.start_time = time.perf_counter()
            return self
        
        def __exit__(self, exc_type, exc_val, exc_tb):
            if _profiling_enabled:
                elapsed = time.perf_counter() - self.start_time
                
                stats = _profile_stats[section_name]
                stats['count'] += 1
                stats['total_time'] += elapsed
                stats['min_time'] = min(stats['min_time'], elapsed)
                stats['max_time'] = max(stats['max_time'], elapsed)
                stats['times'].append(elapsed)
                if len(stats['times']) > 1000:
                    stats['times'].pop(0)
    
    return ProfileSection()


def start_cprofile():
    """Start cProfile profiler."""
    if _cprofile_enabled and _cprofile_profiler:
        _cprofile_profiler.enable()


def stop_cprofile():
    """Stop cProfile profiler."""
    if _cprofile_enabled and _cprofile_profiler:
        _cprofile_profiler.disable()


def get_cprofile_stats(sort_by='cumulative', top_n=20) -> str:
    """
    Get cProfile statistics as a formatted string.
    
    Args:
        sort_by: Sort key ('cumulative', 'time', 'calls', etc.)
        top_n: Number of top functions to show
    
    Returns:
        Formatted statistics string
    """
    if not _cprofile_profiler:
        return "cProfile not initialized"
    
    s = io.StringIO()
    ps = pstats.Stats(_cprofile_profiler, stream=s)
    ps.strip_dirs()
    ps.sort_stats(sort_by)
    ps.print_stats(top_n)
    return s.getvalue()


def get_stats(sort_by='total_time', top_n=None) -> List[Dict[str, Any]]:
    """
    Get profiling statistics.
    
    Args:
        sort_by: Sort key ('total_time', 'count', 'avg_time', 'max_time')
        top_n: Number of top functions to return (None for all)
    
    Returns:
        List of function statistics dictionaries
    """
    results = []
    
    for func_name, stats in _profile_stats.items():
        if stats['count'] == 0:
            continue
        
        avg_time = stats['total_time'] / stats['count']
        
        # Calculate percentiles if we have timing data
        percentiles = {}
        if stats['times']:
            times_sorted = sorted(stats['times'])
            percentiles = {
                'p50': np.percentile(times_sorted, 50),
                'p90': np.percentile(times_sorted, 90),
                'p95': np.percentile(times_sorted, 95),
                'p99': np.percentile(times_sorted, 99),
            }
        
        results.append({
            'name': func_name,
            'count': stats['count'],
            'total_time': stats['total_time'],
            'avg_time': avg_time,
            'min_time': stats['min_time'],
            'max_time': stats['max_time'],
            **percentiles
        })
    
    # Sort results
    if sort_by == 'total_time':
        results.sort(key=lambda x: x['total_time'], reverse=True)
    elif sort_by == 'count':
        results.sort(key=lambda x: x['count'], reverse=True)
    elif sort_by == 'avg_time':
        results.sort(key=lambda x: x['avg_time'], reverse=True)
    elif sort_by == 'max_time':
        results.sort(key=lambda x: x['max_time'], reverse=True)
    
    if top_n:
        results = results[:top_n]
    
    return results


def print_stats(sort_by='total_time', top_n=20):
    """
    Print profiling statistics to console.
    
    Args:
        sort_by: Sort key ('total_time', 'count', 'avg_time', 'max_time')
        top_n: Number of top functions to show
    """
    stats = get_stats(sort_by, top_n)
    
    if not stats:
        print("No profiling data available")
        return
    
    print("\n" + "="*100)
    print(f"PROFILING STATISTICS (sorted by {sort_by}, top {len(stats)})")
    print("="*100)
    print(f"{'Function':<50} {'Count':>8} {'Total(s)':>10} {'Avg(ms)':>10} {'Min(ms)':>10} {'Max(ms)':>10} {'P95(ms)':>10}")
    print("-"*100)
    
    for stat in stats:
        name = stat['name']
        if len(name) > 48:
            name = "..." + name[-45:]
        
        print(f"{name:<50} {stat['count']:>8} "
              f"{stat['total_time']:>10.3f} "
              f"{stat['avg_time']*1000:>10.3f} "
              f"{stat['min_time']*1000:>10.3f} "
              f"{stat['max_time']*1000:>10.3f} "
              f"{stat.get('p95', 0)*1000:>10.3f}")
    
    print("="*100 + "\n")


def get_summary() -> Dict[str, Any]:
    """
    Get summary statistics across all profiled functions.
    
    Returns:
        Dictionary with summary metrics
    """
    if not _profile_stats:
        return {}
    
    total_calls = sum(s['count'] for s in _profile_stats.values())
    total_time = sum(s['total_time'] for s in _profile_stats.values())
    
    return {
        'total_functions': len(_profile_stats),
        'total_calls': total_calls,
        'total_time': total_time,
        'avg_time_per_call': total_time / total_calls if total_calls > 0 else 0
    }


def reset_stats():
    """Clear all profiling statistics."""
    _profile_stats.clear()


def save_stats(filename: str):
    """
    Save profiling statistics to a file.
    
    Args:
        filename: Output filename (JSON format)
    """
    import json
    
    stats = get_stats()
    summary = get_summary()
    
    output = {
        'summary': summary,
        'functions': stats
    }
    
    with open(filename, 'w') as f:
        json.dump(output, f, indent=2)
    
    print(f"Profiling stats saved to {filename}")


# Memory profiling utilities
try:
    import tracemalloc
    _memory_profiling_available = True
except ImportError:
    _memory_profiling_available = False

_memory_tracking_enabled = False


def enable_memory_tracking():
    """Enable memory tracking (requires tracemalloc)."""
    global _memory_tracking_enabled
    if _memory_profiling_available:
        tracemalloc.start()
        _memory_tracking_enabled = True


def disable_memory_tracking():
    """Disable memory tracking."""
    global _memory_tracking_enabled
    if _memory_profiling_available and _memory_tracking_enabled:
        tracemalloc.stop()
        _memory_tracking_enabled = False


def get_memory_snapshot():
    """Get current memory usage snapshot."""
    if not _memory_tracking_enabled:
        return None
    
    snapshot = tracemalloc.take_snapshot()
    top_stats = snapshot.statistics('lineno')
    
    return {
        'current': tracemalloc.get_traced_memory()[0] / 1024 / 1024,  # MB
        'peak': tracemalloc.get_traced_memory()[1] / 1024 / 1024,  # MB
        'top_allocations': [
            {
                'file': stat.traceback.format()[0] if stat.traceback else 'unknown',
                'size_mb': stat.size / 1024 / 1024,
                'count': stat.count
            }
            for stat in top_stats[:10]
        ]
    }


def print_memory_stats():
    """Print memory usage statistics."""
    snapshot = get_memory_snapshot()
    if not snapshot:
        print("Memory tracking not enabled")
        return
    
    print("\n" + "="*80)
    print("MEMORY STATISTICS")
    print("="*80)
    print(f"Current memory usage: {snapshot['current']:.2f} MB")
    print(f"Peak memory usage: {snapshot['peak']:.2f} MB")
    print("\nTop 10 memory allocations:")
    print("-"*80)
    
    for i, alloc in enumerate(snapshot['top_allocations'], 1):
        print(f"{i}. {alloc['file']}")
        print(f"   Size: {alloc['size_mb']:.3f} MB, Count: {alloc['count']}")
    
    print("="*80 + "\n")
