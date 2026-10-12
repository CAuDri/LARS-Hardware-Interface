# ST VL53L5CX Ultra Lite Driver

This directory contains the STSW-IMG023 Ultra Lite Driver core and crosstalk
plugin, API revision `VL53L5CX_2.0.0`, sensor firmware MM1.8.

The files `vl53l5cx_api.*`, `vl53l5cx_buffers.h`, and
`vl53l5cx_plugin_xtalk.*` are vendor sources. `platform.*` is the CAuDri STM32
adapter and intentionally enables only distance, target count, target status,
and reflectance output.

The upstream component repository tag `v1.0.7` still contains API 1.3.9 and
MM1.5 firmware, so it was not used because that would downgrade the KITcar
reference implementation. See `LICENSE.md` for the vendor license.
