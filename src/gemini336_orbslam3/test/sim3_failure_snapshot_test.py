#!/usr/bin/env python3
"""Supervise fixed Sim3 cases; aborts and former fatal snapshots are failures."""

import argparse
from pathlib import Path
import resource
import subprocess
import sys


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def disable_core_dumps():
    # A regression must not fill the disk with child cores. Parent limits and
    # the user's separate runtime/debugger environment are unchanged.
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def check_case(executable, case):
    result = subprocess.run(
        [str(executable), case], capture_output=True, text=True,
        encoding="utf-8", errors="replace", timeout=10,
        preexec_fn=disable_core_dumps, check=False,
    )
    combined = result.stdout + result.stderr
    try:
        phase = "iterate" if case.startswith("ransac_") else "compute"
        require(f"SIM3_CASE name={case} phase={phase}" in result.stdout,
                "Failure before the real solver/iteration call")
        require(result.returncode == 0, f"Child terminated with {result.returncode}")
        for marker in ("SIM3_FAILURE_SNAPSHOT", "SO3::exp failed!", "Sophus ensure failed"):
            require(marker not in combined, f"Unexpected fatal diagnostic: {marker}")

        # C++ verifies the known model, explicit rejection or RANSAC state. This
        # wrapper additionally enforces process survival and quiet error paths.
        prefix = f"SIM3_CHECK name={case} result=PASS outcome="
        checks = [line for line in result.stdout.splitlines() if line.startswith(prefix)]
        require(len(checks) == 1, "Missing/duplicate regression result")
        outcome = checks[0][len(prefix):]
        require(outcome in ("accepted", "rejected", "state_preserved", "ransac"),
                "Unknown regression outcome")
        print(f"PASS {case}: {outcome}, no abort/fatal snapshot", flush=True)
    except AssertionError as error:
        raise AssertionError(
            f"{case}: {error}\n--- child stdout ---\n{result.stdout}"
            f"\n--- child stderr ---\n{result.stderr}"
        ) from error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("solver", type=Path, help="Built sim3_solver_test executable")
    parser.add_argument("cases", nargs="+", help="Cases supplied by the CMake registration")
    arguments = parser.parse_args()
    if len(arguments.cases) != len(set(arguments.cases)):
        parser.error("Duplicate regression case")
    executable = arguments.solver.resolve(strict=True)
    failures = 0
    for case in arguments.cases:
        try:
            check_case(executable, case)
        except (AssertionError, OSError, subprocess.TimeoutExpired) as error:
            failures += 1
            print(f"FAIL {case}: {error}", file=sys.stderr, flush=True)
    print(f"Safety checks: {len(arguments.cases) - failures}/{len(arguments.cases)} passed", flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
