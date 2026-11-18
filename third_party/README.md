Vendor sources are included so normal builds do not access the network.

The complete upstream archive URL, version, and SHA-256 are recorded in
`../scripts/dependencies.lock.json`. `python3 scripts/bootstrap_deps.py --force`
restores the same minimal source selection and verifies each archive before
extracting it. It retains the upstream license files in each dependency directory.

* ST STM32H7 HAL driver v1.11.3 (BSD-3-Clause).
* ST CMSIS device H7 v1.10.5 (Apache-2.0).
* ARM CMSIS Core 5.9.0 (Apache-2.0).
* TinyUSB 0.18.0 (MIT).

No vendor source modifications have been made. Only the headers and sources
needed by this STM32H743 application are retained.
