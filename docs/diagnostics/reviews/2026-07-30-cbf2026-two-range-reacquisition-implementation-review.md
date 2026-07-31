# CBF2026 two-range reacquisition implementation review

## Decision

The final pre-commit implementation snapshot is accepted with:

- Critical: `0`;
- Important: `0`; and
- Minor: `0`.

Every Important finding recorded below received a failing regression or
equivalent RED reproduction,
a bounded implementation repair,
a passing GREEN verification,
and an independent re-review closure.
The cumulative count is `16` Important findings by review event.
It is not a claim of 16 independent root causes:
several later reviews found a narrower bypass or race in an earlier repair.

The reviewed base is
`c69b1db10a8686e88500f29bb1d9ad715461dca2`
on branch `codex/cbf2026-diagnostic`.
The accepted repairs after that commit were reviewed while uncommitted and
are included in the implementation-closure commit.
This decision covers code,
tests,
fixture identity,
transactional publication,
and the verification reported below.
It does not claim that a registered production command,
smoke protocol,
full 140000-row grid,
compact production analysis,
or Stage 2 run was executed.

## Review method and scope

Three independent initial reviews covered:

| Review | Critical | Important | Minor | Finding summary |
| --- | ---: | ---: | ---: | --- |
| Theory and state | `0` | `1` | `0` | The public solver lacked strict validation of `ranging_sigma`. |
| Analyzer and registrar | `0` | `3` | `0` | Raw-origin authority could be forged, the public analyzer cleanup boundary could be bypassed, and a same-inode same-length rewrite after `final_probe` was not detected. |
| Producer and provenance | `0` | `3` | `0` | Terminal publication did not recheck the registered root, registered smoke preflight did not bind complete provenance, and the extractor accepted relocated or root-swapped V4 evidence. |

The reviews inspected only the two-range implementation,
its direct regressions,
the preserved predictive-WNLS suite,
the exact fixture identities,
the direct command-line entry points,
and their transaction boundaries.
The pre-existing untracked `build-diagnostic/` directory was out of scope
and remained untouched.

## Iterative finding ledger

| Review event | Critical | Important | Minor | Cumulative Important | Disposition |
| --- | ---: | ---: | ---: | ---: | --- |
| Initial theory/state review | `0` | `1` | `0` | `1` | Closed |
| Initial analyzer/registrar review | `0` | `3` | `0` | `4` | Closed |
| Initial producer/provenance review | `0` | `3` | `0` | `7` | Closed |
| Wrapper-slot cross-review | `0` | `2` | `0` | `9` | Closed |
| First provenance-fix cross-review | `0` | `2` | `0` | `11` | Closed |
| Corrected analyzer raw-recomputation redesign | `0` | `0` | `0` | `11` | Clean |
| Second provenance-fix cross-review | `0` | `2` | `0` | `13` | Closed |
| Quarantine-helper cross-review | `0` | `2` | `0` | `15` | Closed |
| Zero-unlink tombstone and independent-fsync cross-review | `0` | `1` | `0` | `16` | Closed |
| Atomic no-replace final review | `0` | `0` | `0` | `16` | Clean |

The wrapper-slot review showed that
`object.__getattribute__` could still extract both the minter and a lower
analyzer callable from wrapper slots.
The first provenance follow-up found a completed-manifest residual and a
pathname-child ABA gap.
The second found a public or quarantine stat-to-unlink race and missing
durable directory synchronization after post-rename deletion.
The quarantine-helper follow-up found the same check-to-unlink race inside
quarantine and a control-flow path where root `fsync` failure skipped the
parent `fsync` and continued publication.
The final nonzero review found that ordinary `os.rename` could overwrite a
preoccupied quarantine destination.
The atomic no-replace redesign closed that last race without a clobbering
fallback.

## RED to GREEN dispositions

### Theory and state

RED cases demonstrated acceptance or ambiguous treatment of Boolean,
non-real,
nonfinite,
zero,
and negative ranging sigma values.
GREEN now requires a non-Boolean real that converts to a finite,
strictly positive float.
The normalized float is passed unchanged to both WNLS branch solves.
The tagged private prior remains limited to selecting a discrete two-circle
branch:
it does not enter the FIM,
continuous correction,
or public estimator state.

### Analyzer and registrar

The initial analyzer design exposed process-local authority through closure
and wrapper paths and allowed cleanup to be bypassed.
The accepted redesign removes the secret,
minter,
capability,
registry,
and wrapper entirely.
The public analyzer is the real implementation function.
Immediately before publication,
it recomputes registered or smoke aggregation from retained parsed raw rows,
truth,
protocol,
and representatives,
then compares every semantic result field and the exact row budget.
The raw recomputation context remains local and is not serialized.
The public function's own transaction `finally` covers every post-root
failure.

Registrar RED mutated a pinned output in place after `final_probe` while
preserving inode and length.
GREEN retains read/write descriptors and rereads every byte after the final
external probe,
then compares full descriptor metadata,
size,
and SHA-256 before success.
Failure enters the ownership-aware rollback path.

### Producer, provenance, and fixture extraction

Registered replay preflight now validates the complete raw smoke manifest
and binds the exact root-derived path,
the full process descriptor,
compressed and decompressed content hashes,
protocol identity,
source identities,
and row count.
It repeats raw-record and identity validation after analyzer-smoke
inspection.
Terminal publication checks parent and output-root bindings around completed
publication and retracts an owned completed manifest if a later terminal
check fails.

Retraction uses forensic quarantine plus a zero-unlink tombstone.
Directory durability attempts are independent,
so a root synchronization failure does not suppress the parent attempt.
Atomic no-replace movement uses Darwin `renameatx_np(RENAME_EXCL)` or Linux
`renameat2(RENAME_NOREPLACE)`.
An unsupported platform,
missing symbol,
or syscall failure fails closed.
There is no ordinary-rename or other clobbering fallback,
and a foreign destination is preserved.

The extractor now requires the exact declared V4 `output_root`,
pins its descriptor,
and opens the manifest and process relative to that descriptor with
`dir_fd`.
It checks root and child identity continuity across reading,
hashing,
decompression,
fixture construction,
publication,
and return.
Relocation,
persistent root swap,
and pathname-child ABA regressions fail closed.

The extractor,
replay,
analyzer,
and registrar direct CLIs seed the implementation-root `scripts` namespace
before importing diagnostics.
Their hostile-shadow regressions execute each absolute script from an
unrelated working directory containing a real regular `scripts` package
whose import raises.

## Final verification

| Gate | Result | unittest time | Wall | User | System |
| --- | --- | ---: | ---: | ---: | ---: |
| Focused new implementation | `327 / 327` | `72.382 s` | `73.35 s` | `68.77 s` | `13.54 s` |
| Preserved predictive-WNLS | `230 / 230` | `18.209 s` | `19.09 s` | `33.90 s` | `8.73 s` |
| Full discovery | `864 / 864` | `113.424 s` | `114.37 s` | `124.33 s` | `29.53 s` |
| Python compilation | `6 / 6` | n/a | n/a | n/a | n/a |
| Repository-root CLI help | `4 / 4` | n/a | n/a | n/a | n/a |
| Hostile-shadow CLI help | `4 / 4` | n/a | n/a | n/a | n/a |

The immutable replay-localization calibration source remained:

`0123ed283917f6f9c6a005df4b8b7928e927d3aa4e098e5740816f272e24e4b8`

The V4 manifest,
compressed process,
and decompressed process remained respectively:

- `123b365662273cf768fc9f35530013b2af8fddb6d5ead7d7c1bc9436a2bb8223`;
- `9710d1a5792e682bb272d0bb3305162180d1e0b9a0636647d6e071d1782a699b`;
  and
- `7047989804fd03bd4cadffac8212349b7183857623ac201d14ef2fc3e30ee2e1`.

The approved fixture mechanism is still `3432` bytes with SHA-256
`9febff2393017b7b0fbd1a02dd76a13d1d70639f2b176df236931fe29a8601c7`.
The fixture manifest is `863` bytes with SHA-256
`cb704d10d439dac55170ce14a9feef27c92cd60ec2a95ccb4c47a2cec73a722d`.
The final extractor is `26185` bytes with SHA-256
`4c35e8c0ce8858ff247f07639b9b190f3196d31d80721b12b90bc507c24b8775`.
The final fixture mechanism remains byte-for-byte unchanged.

All six registered,
smoke,
development,
and analysis roots were absent:

- `/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-a`;
- `/private/tmp/cbf2026-two-range-reacquisition-smoke-v1-b`;
- `/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-a`;
- `/private/tmp/cbf2026-two-range-reacquisition-smoke-analysis-v1-b`;
- `/private/tmp/cbf2026-two-range-reacquisition-development/v1`; and
- `/private/tmp/cbf2026-two-range-reacquisition-analysis/v1`.

The final implementation diff passed `git diff --check`.

## Claim boundaries

The analyzer's reviewed boundary rejects candidate-result,
projection,
context,
and reuse substitution through the real public flow.
It does not protect against arbitrary same-process mutation of globals or
bytecode,
tracing or frame access,
or direct monkeypatching of the validator.
That requires a process-isolation boundary.

The reviewed no-replace transaction protects owned evidence during the
publication and retraction flow.
It does not protect files from privileged mutation after return.
CLI help and unit tests are not production runs.

The scientific limitations remain one preserved truth trajectory,
an estimator outside the controller,
a diagonal marginal-covariance FIM approximation,
unmodeled shared-ancestor cross-covariances,
a conditional modeled three-standard-deviation radius,
`243 / 7000` applied source component-bound violations,
and closed paper-edit and bounded-input Stage 2 gates.
The accepted snapshot therefore does not establish cross-trajectory
reliability,
closed-loop estimator robustness,
unconditional CBF safety,
or a deterministic localization-error guarantee.
