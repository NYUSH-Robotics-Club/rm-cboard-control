# Repository guidance

This is STM32F407 RoboMaster firmware. Read the relevant current document in
`docs/README.md` for the task at hand. `docs/project/PROJECT_MEMO.md` holds a
short list of durable constraints and open questions; consult it for control,
hardware, architecture, and release work. Do not load historical Git revisions
as current requirements.

- The current startup initializes hardware, then starts the static FreeRTOS control
  task in `runtime/rtos/`. Supported configurations are `infantry_standard` and
  `sentry_swerve`.
- Keep the dependency direction in `docs/architecture/overview.md`. Unknown CAN
  IDs, payloads, motor limits, geometry, and Jetson protocol details must stay
  explicitly unsupported until specified and validated.
- The low-level baseline is frozen: `Inc/`, `Src/`, `Drivers/`, `Middlewares/`,
  `NYUSH_Infantry.ioc`, startup assembly, linker script, and
  `cmake/stm32cubemx/CMakeLists.txt`. Change these only when the user explicitly
  authorizes that scope. Earlier one-time exceptions do not carry forward.
- For behavior changes, run the relevant host checks and build affected robot
  configurations. Do not flash or move motors as an automatic check. Report
  compilation, host tests, board verification, and observed behavior separately.
- Update a maintained document only when its current instructions or facts
  change. Keep dated logs and experiment narratives out of current guides; Git
  history preserves old investigations. Do not append a task diary to the memo.
- In changed upper-layer code, explain non-obvious units, ownership, and failure
  behavior. Avoid comments that merely restate the code.
