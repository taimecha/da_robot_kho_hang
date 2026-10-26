#!/usr/bin/env python3
"""Summarize two-AGV pickup/delivery events from cargo_live_view.py.

Example: python3 tools/analyze_cargo_timing.py RUN/cargo_events.jsonl --environment real
         python3 tools/analyze_cargo_timing.py RUN/cargo_events.jsonl --csv RUN/cargo_times.csv
Only a complete started -> loaded -> delivery_arrived cycle is counted as delivered.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from statistics import median


EVENTS = ('started', 'observation_arrived', 'measured', 'pickup_arrived',
          'loaded_confirmed', 'delivery_arrived', 'unloaded_confirmed')
COLUMNS = ('environment', 'run_id', 'car', 'cycle', 'status', 'deliver_col',
           'T_to_observe_s', 'T_measure_s', 'T_to_pickup_s', 'T_wait_load_s',
           'T_pickup_s', 'T_delivery_s', 'T_total_s', 'T_to_unload_confirmed_s')


def duration(milestones, end, start):
    if start not in milestones or end not in milestones:
        return None
    value = milestones[end] - milestones[start]
    return round(value, 3) if value >= 0 else None


def summarize(records, environment):
    groups = defaultdict(dict)
    info = defaultdict(dict)
    for record in records:
        event = record.get('event')
        if event not in EVENTS or record.get('cycle') is None:
            continue
        key = (record['run_id'], record['car'], int(record['cycle']))
        # Keep the first accepted occurrence; retries remain visible in the raw log.
        groups[key].setdefault(event, float(record['t_monotonic']))
        if event in ('measured', 'loaded_confirmed', 'delivery_arrived'):
            info[key].update({k: record[k] for k in ('deliver_col',) if k in record})
    rows = []
    for (run_id, car, cycle), m in sorted(groups.items()):
        ordered = [m[event] for event in EVENTS if event in m]
        complete = all(event in m for event in EVENTS[:6]) and ordered == sorted(ordered)
        status = 'delivered' if complete else 'incomplete_or_invalid'
        rows.append(dict(environment=environment, run_id=run_id, car=car, cycle=cycle,
                         status=status, deliver_col=info[(run_id, car, cycle)].get('deliver_col'),
                         T_to_observe_s=duration(m, 'observation_arrived', 'started'),
                         T_measure_s=duration(m, 'measured', 'observation_arrived'),
                         T_to_pickup_s=duration(m, 'pickup_arrived', 'measured'),
                         T_wait_load_s=duration(m, 'loaded_confirmed', 'pickup_arrived'),
                         T_pickup_s=duration(m, 'loaded_confirmed', 'started'),
                         T_delivery_s=duration(m, 'delivery_arrived', 'loaded_confirmed'),
                         T_total_s=duration(m, 'delivery_arrived', 'started') if complete else None,
                         T_to_unload_confirmed_s=(
                             duration(m, 'unloaded_confirmed', 'started') if complete else None)))
    return rows, groups


def read_events(path):
    records = []
    for line_number, line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(), 1):
        if not line.strip():
            continue
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as error:
            raise ValueError(f'{path}:{line_number}: invalid JSON: {error}') from error
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('events', help='cargo_events.jsonl from cargo_live_view.py')
    parser.add_argument('--environment', choices=('sim', 'real'), required=True)
    parser.add_argument('--csv', help='output per-cycle CSV; defaults to next to events file')
    args = parser.parse_args()
    rows, groups = summarize(read_events(args.events), args.environment)
    out = Path(args.csv) if args.csv else Path(args.events).with_name('cargo_times.csv')
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    print(f'CSV: {out} ({len(rows)} cycles)')
    for car in sorted({row['car'] for row in rows}):
        subset = [row for row in rows if row['car'] == car]
        good = [row for row in subset if row['status'] == 'delivered']
        totals = [row['T_total_s'] for row in good]
        print(f'{car}: {len(good)}/{len(subset)} delivered; '
              f'T_total median={median(totals):.3f} s, range={min(totals):.3f}–{max(totals):.3f} s'
              if totals else f'{car}: 0/{len(subset)} delivered')
    # A batch spans two cars' cycles with the same run_id and cycle number.
    batches = defaultdict(dict)
    delivered_keys = {(row['run_id'], row['car'], row['cycle']) for row in rows
                      if row['status'] == 'delivered'}
    for (run_id, car, cycle), events in groups.items():
        if (run_id, car, cycle) in delivered_keys:
            batches[(run_id, cycle)][car] = events
    for (run_id, cycle), cars in sorted(batches.items()):
        if set(cars) == {'agv1', 'agv2'}:
            start = min(m['started'] for m in cars.values())
            finish = max(m['delivery_arrived'] for m in cars.values())
            print(f'2-car batch {run_id} cycle {cycle}: {finish-start:.3f} s')


if __name__ == '__main__':
    main()
