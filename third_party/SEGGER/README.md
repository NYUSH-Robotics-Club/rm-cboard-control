# SEGGER RTT dependency

The three files in `RTT/` are unmodified sources from
[SEGGERMicro/RTT](https://github.com/SEGGERMicro/RTT/tree/4d8feab3150f86f37a9d323ddc88d6cdf5673072).
The upstream license is retained in `LICENSE.md`; commit and SHA-256 checksums
are recorded in `source.json`.

`Config/SEGGER_RTT_Conf.h` is the project's existing configuration. It retains
two up channels and selects `RTT_USE_ASM=0` because CMake compiles the portable
C implementation. Channel 1 carries the existing dashboard frames with
nonblocking skip-on-full behavior. No syscall retargeting or startup assembly
is included.
