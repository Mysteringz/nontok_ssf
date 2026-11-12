#!/usr/bin/env python3
"""Summarises a day's mission log (PLAN.md section 13.13). No ROS needed.

    python3 tools/mission_report.py                  today's log in ~/nontok_logs
    python3 tools/mission_report.py path/to/2026-11-20.jsonl
"""
import collections
import datetime
import json
import os
import sys


def summarise(lines):
    started = ended = 0
    results = collections.Counter()
    faults = collections.Counter()
    durations = []
    start_time = None
    for line in lines:
        try:
            event = json.loads(line)
        except ValueError:
            continue
        kind = event.get('event')
        when = datetime.datetime.fromisoformat(event['time']) if 'time' in event else None
        if kind == 'mission_start':
            started += 1
            start_time = when
        elif kind == 'mission_end':
            ended += 1
            results[event.get('result', 'OK')] += 1
            if start_time and when:
                durations.append((when - start_time).total_seconds())
            start_time = None
        elif kind == 'fault':
            faults[event.get('reason', '?')] += 1
            start_time = None
    return {'started': started, 'ended': ended, 'results': dict(results), 'faults': dict(faults),
            'mean_duration_s': sum(durations) / len(durations) if durations else None}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        '~/nontok_logs/%s.jsonl' % datetime.date.today().isoformat())
    with open(path) as handle:
        report = summarise(handle)
    delivered = report['results'].get('OK', 0)
    print('Missions started:   %d' % report['started'])
    print('Item delivered:     %d' % delivered)
    for result, count in sorted(report['results'].items()):
        if result != 'OK':
            print('Returned, %-16s %d' % (result + ':', count))
    for reason, count in sorted(report['faults'].items()):
        print('Fault, %-19s %d' % (reason + ':', count))
    if report['mean_duration_s'] is not None:
        print('Mean mission time:  %.0f s' % report['mean_duration_s'])
    if report['started']:
        print('Success rate:       %.0f %%  (CP11 needs 8 of 10)' % (100.0 * delivered / report['started']))


if __name__ == '__main__':
    main()
