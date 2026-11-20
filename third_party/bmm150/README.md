# Bosch BMM150 SensorAPI

Upstream: https://github.com/boschsensortec/BMM150_SensorAPI
Version: v2.0.0 (2020-06-03); BSD-3-Clause notices retained in each source.
Retrieved 2026-11-20 from the jsDelivr GitHub mirror (master).
Only the pure `bmm150_aux_mag_data` compensation path is used. All bus access
belongs to our non-blocking state machine; the API callbacks reject I/O.
Compiled with BMM150_USE_FLOATING_POINT, output in microtesla.

SHA256 (unmodified upstream files):
- `bmm150.c`: `477270b71a7c7b47acf47a21a7bc0b972bdf8deb021a6597b44b6cf2a3f1e926`
- `bmm150.h`: `8ab084b4d7217b3b99b3da552b313b19f0556ae3612d7cfcc0ec6bcef44e1494`
- `bmm150_defs.h`: `4a5058a7461b7b0f788f28cf7a4fc633a4cb73b4df8b3cafb7b8eb910ea80a24`
