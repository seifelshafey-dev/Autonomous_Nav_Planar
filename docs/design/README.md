# Design notes

These pages explain why the planner is built the way it is. Each one derives the bounds it relies on, ties every formula to the code symbol that implements it and the test that checks it, and ends with its limits.

The figures are drawn from the real code and can be redrawn: see [the viz tools](../../src/path_planner/tools/viz/README.md).

1. [Motion primitives](primitives.md): the twelve `(v, ω, dt)` motions the search chooses from and how they are built from the robot configuration.
2. [The swept-cell cover](swept-cover.md): the cached list of cells each motion can touch, and why a clear list proves the motion is clear.
3. [Cost and heuristic](cost-and-heuristic.md): what the search minimises, how close-to-wall motion is priced, and where the estimate of the remaining cost is wrong.
4. [Navigation](navigation.md): the modes, the numbers the navigator works out for itself, the stall rule, and the short move out of a blocked start.
5. [The scan safety guard](safety-guard.md): how the last check before the wheels uses the newest scan to block a command.
