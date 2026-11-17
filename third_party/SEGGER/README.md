# SEGGER RTT dependency

The three files in `RTT/` are unmodified sources from
[SEGGERMicro/RTT](https://github.com/SEGGERMicro/RTT/tree/4d8feab3150f86f37a9d323ddc88d6cdf5673072).
The upstream license is retained in `LICENSE.md`; commit and SHA-256 checksums
are recorded in `source.json`.

`Config/SEGGER_RTT_Conf.h` selects the portable C implementation (`RTT_USE_ASM=0`).
Channel 0 is bidirectional: an 8192-byte up buffer carries whole JSON-line records,
and a 512-byte down buffer carries text commands. Output skips whole records when
full; the diagnostic terminal retains command replies for retry and counts dropped
CAN event records. Only foreground code calls RTT. No syscall retargeting is used.
