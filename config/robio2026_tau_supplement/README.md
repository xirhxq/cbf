# ROBIO 2026 closed-loop tau check

Configs for the threshold check in Sec. VI-C of *Joint-Feasibility Margin Feedback for Bounded-Input Second-Order Control Barrier Functions* (ROBIO 2026).

All 12 runs use the first paired initial condition of the main study (seed 20262701), low and critical tasks, and tau in {10, 12, 14, 16, 18} m/s^2, plus one hard-QP baseline per task. Every other setting matches the main study (M = 8, H = 1, T_s = 0.5 s).

Build the `Swarm` target as described in the top-level README (Gurobi enabled), then run each config, for example:

```bash
./build/Swarm config/robio2026_tau_supplement/05-I01-critical-proposed-tau10.json
```

The `independent-tau-study` field admits tau other than 14 for this task; without it, the post-detection response task keeps tau = 14.

These files differ from the archived run configs only in `output_path`, which is now relative. The archived SHA-256 of each original config:

| Config | Controller | tau | Archived SHA-256 |
|---|---|---|---|
| `00-I01-critical-proposed-tau14.json` | proposed | 14 | `203f6b62b275a7a3458ef9fbc7786d3d8f06499dccf368acf2548962097df4f4` |
| `01-I01-low-proposed-tau14.json` | proposed | 14 | `2d19d1ee33164f36c224f853d2e4e4dc8d31af19b854b560763abbff7cc0c683` |
| `02-I01-low-legacy-tau14.json` | legacy | 14 | `f692225ff3015582e5528b71dd0d532a8b82a4fe1998ab518de77dc72e9a5eb6` |
| `03-I01-critical-legacy-tau14.json` | legacy | 14 | `c18836325120dc07fceb9aa5623a79295768716bf12c89cc9017b8b7a2c8cc2a` |
| `04-I01-low-proposed-tau10.json` | proposed | 10 | `4d979a3552f02a475da0fbf31e54ef3e857a35afb6b8e004ff6223e6d7776a76` |
| `05-I01-critical-proposed-tau10.json` | proposed | 10 | `e60e62cce2d1e50e180cc1034bb271676ac83936c98522389455730a1aed0bb4` |
| `06-I01-low-proposed-tau12.json` | proposed | 12 | `7fc7fe185ef24949b4b30279d5725709f7667f5ed90230a67f9eac44581ae427` |
| `07-I01-critical-proposed-tau12.json` | proposed | 12 | `8f6c6554cad9a87daad96b87aea0f25ad34a8b03a89b59c642d1095bae9cc6ad` |
| `08-I01-low-proposed-tau16.json` | proposed | 16 | `38c674dafee009f78171944a96688358f12620a8fd472900207e0c3450bb2800` |
| `09-I01-critical-proposed-tau16.json` | proposed | 16 | `7d24792ad70e09db65fbd9e50e6c24b8a4d989dbb3a02852328652166a5ea3cc` |
| `10-I01-low-proposed-tau18.json` | proposed | 18 | `3bb2599df4e00cb9a0db66f107faa948d04a3800a491cea3543f8855465588ec` |
| `11-I01-critical-proposed-tau18.json` | proposed | 18 | `12ac922ec41d70a2989d46a603cbaa8c0f26f8c718bd64049b2004dd77e804c5` |
