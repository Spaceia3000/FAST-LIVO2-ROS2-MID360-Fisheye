#!/usr/bin/env python3
"""Compare captured camera ns stamps and native six-decimal logging with bag records."""
import argparse, csv, hashlib, json, pathlib, re
ap=argparse.ArgumentParser();ap.add_argument('run',type=pathlib.Path);ap.add_argument('expected_csv',type=pathlib.Path);a=ap.parse_args()
expected=[int(x['header_ns']) for x in csv.DictReader(a.expected_csv.open())]
actual=[int(x['stamp_ns']) for x in csv.DictReader((a.run/'input_stamps.csv').open()) if x['topic']=='/camera/camera/color/image_raw']
logged=re.findall(r'Get image, its header time: ([0-9.]+)',(a.run/'frontend.log').read_text(errors='replace'))
rounded=[format(t/1e9,'.6f') for t in expected]
result=dict(scope='Camera observer compared at exact nanoseconds and exact order; native callback log compared at its printed six-decimal precision and exact order; logging does not independently expose nanoseconds',expected_count=len(expected),observer_count=len(actual),native_log_count=len(logged),observer_exact_ns_sequence=actual==expected,native_log_exact_printed_sequence=logged==rounded,observer_missing_ns=sorted(set(expected)-set(actual)),observer_extra_ns=sorted(set(actual)-set(expected)),native_missing_printed=sorted(set(rounded)-set(logged)),native_extra_printed=sorted(set(logged)-set(rounded)),expected_csv=str(a.expected_csv),expected_csv_sha256=hashlib.sha256(a.expected_csv.read_bytes()).hexdigest(),input_csv_sha256=hashlib.sha256((a.run/'input_stamps.csv').read_bytes()).hexdigest(),frontend_log_sha256=hashlib.sha256((a.run/'frontend.log').read_bytes()).hexdigest())
result['passed']=len(expected)==4502 and result['observer_exact_ns_sequence'] and result['native_log_exact_printed_sequence']
(a.run/'camera_receipt_integrity.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
