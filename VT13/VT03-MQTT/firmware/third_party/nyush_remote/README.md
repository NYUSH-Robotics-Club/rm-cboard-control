# nyush-rm-control remote receive chain

Source: https://github.com/NYUSH-Robotics-Club/nyush-rm-control
Commit: `2b7ca720e856f1efda9f6de6259faea42654720d`.
The eight C/header files and MIT license are copied unchanged. `source.json` records
SHA-256 hashes of the copied bytes. The build compiles these files through thin wrappers.

Integration outside the copied sources:

- `bsp/remote/nyush_usart.c` renames the shared HAL callbacks so USART1 WT61C keeps its handler.
  A wrapper records ReceiveToIdle return values without adding aborts or retry policy.
- `modules/remote/nyush_remote.c` converts the original decoded struct into the existing
  message contract. The original keyboard state and combination counters remain in the decoder.
- `modules/remote/remote_control.c` publishes only complete 18-byte DBUS events. The original
  decoder still sees partial events, but partial data cannot publish robot commands.
- The original daemon runs every 10 ms from the bare-metal dispatch loop; no RTOS is started.
  It keeps the upstream initial 10 counts and direct receive retry, with no Dart 200 ms abort loop.
- Upstream warnings/errors increment counters instead of emitting blocking IRQ logs.
  The unused daemon DWT/buzzer includes use empty compatibility headers scoped to these wrappers.
- Upstream CRC symbols are prefixed to coexist with the existing vision CRC implementation.
- USART3 100000/9B+EVEN, PC11 AF7, DMA1 Stream1 CH4 NORMAL/LOW, and receiver IRQ priority 5
  match the reference. The project's global 72 MHz clock, HAL 1.8.5 and other peripherals stay in place.
- The project command controller still disables stale remote commands after 200 ms.

`tests/host/test_remote_chain.c` executes the copied registry, decoder and daemon with
fake HAL hardware, then checks the real message bus and command controller. It covers
TC/IDLE, callback snapshots, BUSY without abort, channel/keyboard mapping, timeout, UART
error and reconnection. Host success does not establish electrical reception on the board.
