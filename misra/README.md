# MISRA C:2012 deviation register

CI runs the cppcheck MISRA addon over the target code (`src/`, without the host-only `hal/host`) and **fails on any finding not listed here**. File- and project-scoped deviations are in `suppressions.txt`; single-line ones are inline `// cppcheck-suppress <rule> ; DEV-xxx: <reason>` comments, and CI rejects any inline suppression without a `DEV-` number. Register, list and comments change together, in the same PR, with a reason.

| ID | Rule (category) | Scope | Reason | Compensating measure |
|---|---|---|---|---|
| DEV-001 | 15.5 single point of exit (advisory) | project | Early returns for argument and state checks keep the code flat and readable; a single exit would need flag variables and deeper nesting. | Early returns are argument/state guards; every return path is exercised by the host tests (coverage floors in CI); `-Wall -Wextra -Werror` plus cppcheck warning checks are blocking. |
| DEV-002 | 8.7 no external linkage if used in one unit (advisory) | `app/safety_app.c` (`safety_app_init()`, `safety_app_step()`, `safety_app_status()`) | The application entry points are called by the board `main()` (red zone, with the CubeMX project, D-118) and the host tests, neither of which the addon scans. | Revisit when the board `main()` lands: drop the deviation for the app entry points. |
| DEV-003 | retired by D-118 | none | Was the `union` in the H755 mailbox region type (19.2); the mailbox is gone with the platform-CAN link. The number is not reused. | none |

CI builds cppcheck 2.22.0 and caches it. Before the MISRA step it checks that the addon reports 10.8, 15.6 and 17.7 on `misra/canary.c`, so a missing or broken addon never passes as a clean run.

**Generated code:** `external/vehicle-defs/gen/c/safety/` is excluded here (`--suppress=*:*external/*`). It is MISRA-checked, blocking, in vehicle-defs CI (its own register in `misra/README.md` there).

**Local runs (macOS):** Homebrew's python3 may lack `expat`; add `--addon-python=/usr/bin/python3` locally (never in `ci.yml`).
