# LEFT L20-10/G20 conversion evidence

Source, commit and library hash match manifest.json and l20_10_left_factory.json. g20_left_raw_to_rad.json contains the 256×16 outputs of the pure factory function for LEFT=0x28. No hand object, callbacks or bus were constructed. libcanbus was replaced by an empty stub during offline export. Runtime uses only Python arithmetic; it never loads this SDK library.

The dedicated LEFT factory function differs from RIGHT on all 16 axes. Its ranges match the archive/lhg20/left model (not the old L20 left model). The spelling/zero velocity fixes are isolated and recorded in teleop/models/l20_10_left/manifest.json. No right-side sign adapter was applied.

This is manufacturer conversion evidence, not a per-device physical angle calibration. Basic swapped full-hand imitation was accepted by the operator on 2026-10-22; quantitative angle alignment remains unmeasured.
