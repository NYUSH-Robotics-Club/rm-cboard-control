# Project Working Agreement

Before changing this repository, read `docs/project/PROJECT_MEMO.md` completely. Treat it as
the durable project memory for architecture decisions, hardware unknowns,
compatibility requirements, and unfinished work.

For every task:

1. Read `docs/project/PROJECT_MEMO.md` before inspecting or editing implementation files.
   During the current yaw investigation session, also read
   `docs/project/YAW_DIAGNOSTIC_MEMO.md` before every task, including log analysis.
   Update that memo before finishing work on this issue; keep observations tied to
   their recording and distinguish source changes from flashed firmware.
2. Preserve the dependency direction documented there.
3. Do not invent CAN identifiers, feedback layouts, baud rates, or protocols for
   hardware marked as unknown. Add an explicit unsupported adapter instead.
4. The low-level baseline is frozen by user requirement. Never edit `Inc/`,
   `Src/` (including `Src/main.c`), `Drivers/`, `Middlewares/`, the `.ioc` file,
   startup assembly, linker script, or `cmake/stm32cubemx/CMakeLists.txt` unless
   the user explicitly removes this restriction in a later request.
5. Run the most relevant available build or static checks after changes.
6. Before finishing, update `docs/project/PROJECT_MEMO.md` with decisions, changed interfaces,
   validation performed, and remaining unknowns. Keep it concise and current.
7. Follow `docs/project/COMMENTING_STANDARD.md`. New or changed upper-layer code
   must use plain comments that explain purpose, units, ownership, and failure
   behavior. Do not add comments to the frozen low-level baseline.
