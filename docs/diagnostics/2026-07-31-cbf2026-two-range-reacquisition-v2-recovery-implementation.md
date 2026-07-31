# CBF2026 two-range reacquisition v2 recovery implementation closure

Date: 2026-07-31

Worktree: `/private/tmp/cbf2026-diagnostic`

Branch: `codex/cbf2026-diagnostic`

Implementation range: `12c00ff63c5757a57053e6f026e0e3cf2c9aa07b` through
`f5e1b8ff52b2590f3b8f3c18e5e81587923b9ca6`.

## Immutable v1 failure boundary

Protocol v1 is a terminal integration failure, not a negative scientific
result.  Its smoke-A invocation raised
`ValueError: protocol source members differ from contract` before output-root
allocation.  Protocol v1 has `retry_allowed: false` for every invocation;
smoke A was not retried, no later v1 smoke or analyzer command ran, and no v1
root was reused, replaced, overwritten, or repurposed.

The v1 terminal-failure record remains authoritative: Task 8 is `FAIL`, the
registered replay and analyzer are unauthorized, and the paper gate is
`CLOSED`.  The v2 source recovery does not reopen either gate.  Registered
execution remains `CLOSED` pending a separately created v2 protocol,
preflight, smoke review, and external authorization record; the paper gate
remains `CLOSED`.

## Test-first evidence

Task 1 first reproduced the defect using the public replay entry point and a
serialized full protocol:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition.ProducerLifecycleTests.test_serialized_full_protocol_smoke_accepts_exact_local_source_subset \
  -v
```

Before the Task 1 production change this ran one test in `0.029s` and failed
with the expected pre-root `ValueError`.  After the change it ran one test in
`0.038s` and passed.  The current closure rerun of that exact command passed:

```text
Ran 1 test in 0.032s
OK
```

Task 1 GREEN evidence also recorded these exact commands and results:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition.ProducerLifecycleTests -v
# Ran 38 tests in 0.582s; OK

conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition -v
# Ran 62 tests in 1.256s; OK
```

Task 2 RED was run before the namespace implementation:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_two_range_reacquisition -v
# Ran 50 tests in 13.640s; FAILED (failures=14, errors=5)

conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition -v
# Ran 63 tests in 1.406s; FAILED (failures=1)

conda run -n cbf_env python -m unittest \
  tests.test_analyze_two_range_reacquisition -v
# Ran 172 tests in 3.796s; FAILED (failures=31)
```

Those failures were the expected v1-to-v2 literal contract mismatches; they
did not execute a production protocol.  The recorded Task 2 GREEN commands
and results were:

```bash
conda run -n cbf_env python -m unittest \
  tests.test_register_two_range_reacquisition -v
# Ran 50 tests in 24.456s; OK

conda run -n cbf_env python -m unittest \
  tests.test_replay_two_range_reacquisition -v
# Ran 63 tests in 0.926s; OK

conda run -n cbf_env python -m unittest \
  tests.test_analyze_two_range_reacquisition -v
# Ran 172 tests in 3.968s; OK

conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -v
# Ran 870 tests in 152.566s; OK
```

This closure reran the exact full-discovery command:

```bash
conda run -n cbf_env python -m unittest discover \
  -s tests -p 'test_*.py' -v
```

It passed `870` tests in `164.895s`.  `git diff --check` exited `0` with no
output before this report was added.  Task 2 also recorded a clean six-file
`py_compile`, direct replay/analyzer/registrar `--help` checks, and three
shadow/direct-bootstrap regressions (`3/3` in `0.753s`).

## Source-binding recovery contract

The complete global declaration remains exactly these ordered 11 members:

```text
implementation_plan
two_range_reacquisition_source
predictive_wnls_source
fixture_extractor_source
replay_source
analyzer_source
registrar_source
mechanism_fixture
mechanism_fixture_manifest
truth_data
input_manifest
```

For `smoke_a`, the runtime observation is deliberately the exact ordered
five-member local subset:

```text
two_range_reacquisition_source
predictive_wnls_source
replay_source
mechanism_fixture
mechanism_fixture_manifest
```

`_validate_protocol_source_binding(...)` now rejects a missing, extra, or
reordered global declaration; requires every global identity to use the exact
`(path, device, inode, size, mtime_ns, sha256)` field order; requires the
invocation-specific local member order; and compares each observed local
identity to its declared identity.  The regression independently derives
identities with `Path.stat()` and SHA-256, serializes and reloads the full
11-member protocol, enters the real public replay function, and confirms the
completed 18-row smoke manifest.  Global membership/order and identity-field
drift, plus local membership/order/identity drift, fail before root
allocation.  Registered replay independently rereads all 11 live declared
sources, so all-source registered validation remains strict.

## v2 provenance namespace

The current production identities are exactly:

| Contract | Identity |
| --- | --- |
| Protocol schema | `cbf2026-two-range-reacquisition-protocol-v2` |
| Registration/authorization schema | `cbf2026-two-range-reacquisition-registration-v2` |
| Protocol ID | `cbf2026-two-range-reacquisition-v2` |
| Raw schema | `cbf2026-two-range-reacquisition-raw-v1` |
| Analysis schema | `cbf2026-two-range-reacquisition-analysis-v1` |

The six current v2 roots are:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v2-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v2-b
/private/tmp/cbf2026-two-range-reacquisition-development/v2
/private/tmp/cbf2026-two-range-reacquisition-analysis/v2
```

The v2 protocol and authorization declarations are respectively
`docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json`
and
`docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json`.
The preflight-review and smoke-report declarations are likewise v2-dated and
v2-named.  All six literal command arrays bind their protocol, raw-root,
output-root, and authorization tokens to this v2 namespace.  The registrar
also binds `implementation_plan` to
`docs/superpowers/plans/2026-07-31-cbf2026-two-range-smoke-v2-recovery.md`.

Raw and analysis remain v1 at the field level because the recovery corrects
only source-binding semantics and the provenance namespace.  `ROW_FIELDS`,
raw manifest/identity validation, `ANALYSIS_FIELDS`, nested analysis field
orders, analysis validation, experiment constants, comparators, fixtures,
and thresholds were not changed.  In particular,
`measurement_seed_contract` remains `cbf2026-range-v1`.  Reversion would
therefore misstate an unchanged raw or analysis schema as a new scientific or
evidence format; only the enclosing registered protocol/authorization
identity needs v2 provenance.

## Commits and exact changed files

| Range | Commit | Subject | Changed files |
| --- | --- | --- | --- |
| v1 failure baseline | `12c00ff63c5757a57053e6f026e0e3cf2c9aa07b` | `docs(diagnostics): record two-range smoke failure` | baseline only |
| Task 1 | `a4b3f6b962b701f8570effd942535fd5d12f8f68` | `fix(diagnostics): validate smoke source subset` | `docs/superpowers/plans/2026-07-31-cbf2026-two-range-smoke-v2-recovery.md`; `scripts/diagnostics/replay_two_range_reacquisition.py`; `tests/test_replay_two_range_reacquisition.py` |
| Task 2 | `f5e1b8ff52b2590f3b8f3c18e5e81587923b9ca6` | `fix(diagnostics): retire two-range protocol v1` | `scripts/diagnostics/register_two_range_reacquisition.py`; `scripts/diagnostics/replay_two_range_reacquisition.py`; `tests/test_analyze_two_range_reacquisition.py`; `tests/test_register_two_range_reacquisition.py`; `tests/test_replay_two_range_reacquisition.py` |

## Preservation and absence audit

The four preserved v1 documents were not modified.  Their current bytes and
SHA-256 values are:

| Document | Bytes | SHA-256 |
| --- | ---: | --- |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.json` | 69464 | `8e01df56623bb2e759fb37108ddc68fbc41e301b8305ade7322e212ae6e3ea16` |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1.md` | 4137 | `c9e3930d22355747ee12e96303c1702aa0385de2881f6b430a72030b89d7e0ff` |
| `docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-protocol-v1-review.md` | 14948 | `6ac2da2d1fdcd173de04d1e9658560ac25439e79d03f1b5649d63f62da805cfa` |
| `docs/diagnostics/2026-07-30-cbf2026-two-range-reacquisition-smoke.md` | 13904 | `404de547db5fb7711f1fa95132cff8399b3fd432a9871c4e600121784f0af8db` |

A read-only `-e`/`-L` audit found all 12 execution roots absent.  The six
retired v1 roots are:

```text
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a
/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b
/private/tmp/cbf2026-two-range-reacquisition-development/v1
/private/tmp/cbf2026-two-range-reacquisition-analysis/v1
```

The remaining six absent paths are the current v2 roots listed above.  The
check includes broken symlinks.  Both authorization records are absent:

```text
docs/diagnostics/reviews/2026-07-30-cbf2026-two-range-reacquisition-registered-authorization.json
docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json
```

No protocol command was executed in this recovery closure.  No registered v2
protocol, authorization, preflight-review, smoke-report, or smoke-review
document was created; these six declared v2 artifacts remain absent:

```text
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.json
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2.md
docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-registered-v2-authorization.json
docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-protocol-v2-review.md
docs/diagnostics/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2.md
docs/diagnostics/reviews/2026-07-31-cbf2026-two-range-reacquisition-smoke-v2-review.md
```

The focused unit test uses an automatically removed `TemporaryDirectory` JSON
fixture solely to verify serialized loading, not a registered protocol
document or command.
No DRA document was edited.

## Closure state

This is an author-side implementation report, not the required independent
implementation review.  It claims no C0/I0/M0 review verdict.  The remaining
work is an independently created review file with Critical `0`, Important
`0`, and Minor `0`, followed only then by the separate DRA append and commits.
