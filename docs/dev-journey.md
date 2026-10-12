# Dev journey

This is the story of what I got wrong while building the planner and what I changed because of it. It is not a full history. It covers the episodes that changed the design, roughly in the order they happened. The current behaviour is in the [architecture pages](architecture/overview.md) and the reasoning behind the bounds is in the [design notes](design/README.md).

## Where it started

In June 2026 I set out to build a hybrid A* planner for a robot that drives with two independent wheels. Hybrid A* means a search that keeps a continuous pose (x, y, heading) instead of hopping between grid points. The moves it tries are a small fixed set of short motions, which I call control primitives: drive straight, drive an arc of a given radius, or turn on the spot. I compute them once at startup, and each one remembers which grid cells the robot's footprint sweeps while executing it.

I chose this over a state lattice, where every motion must start and end on a grid point. A lattice gives exact duplicate detection and reusable collision data, but a robot that can pivot on the spot loses most of what it can do once every motion has to fit the grid. So the state stays continuous. The grid appears in two places only: a coarse key to drop duplicate states, and the cache of swept cells per heading range.

The key matters because two different sequences of motions almost never end at exactly the same continuous state. Without duplicate detection the search treats nearly every state as new, and the count grows twelvefold with each step. So the visited set is keyed by a coarse cell plus a heading bin, while the state itself is never snapped. The cell is a whole number of map cells, 0.15 m in the shipped settings. The cost is one I accepted: the key can drop a candidate that would have led to a cheaper route, so the planner does not promise the cheapest path. A test checks on one scenario that a coarser cell generates fewer nodes and does not lower the cost.

## A heuristic I could not use

The search needs an estimate of the cost still to go. A common choice for a car-like vehicle is the length of a Reeds-Shepp or Dubins path, which assumes the vehicle cannot turn tighter than a minimum radius. They model a vehicle that cannot turn on the spot, and this robot can, so I did not use them. The planner takes the larger of two estimates instead. One is a weighted straight-line distance in position and heading, which I can show never exceeds the true cost. The other is a shortest-route field over the grid that knows about obstacles.

The second one is where I was wrong about my own documentation. I had written that the larger of two lower bounds stays a lower bound, so the combination is safe. The grid route moves in eight directions and cannot follow a straight line at every angle, so on open ground it can exceed the straight-line distance by up to about 8 percent. The planner therefore makes no optimality claim, and I documented the limit. The derivation is in [cost and heuristic](design/cost-and-heuristic.md).

## The corner-cutting model I got wrong

The pure-pursuit follower tracks a path by steering toward a point a fixed distance ahead (the lookahead). An end-to-end test showed the robot cutting corners by about 0.41 m. My model said pure pursuit cuts every curve by an amount that grows with the lookahead, so I planned to shorten the lookahead and size the planner's clearance to match.

While implementing it I measured the follower instead of trusting the model, and the model was wrong. On a smooth arc, pure pursuit tracks the path exactly, whatever the lookahead. The cut only happens at kinks, where one motion ends and the next begins with a different curvature or heading. I also had the tightest arc wrong: it is half the minimum turning radius, 0.15 m. I kept the derived lookahead, which does shrink the cut, and corrected the clearance numbers. The same test then showed about 0.18 m of corner cutting. The clearance remains a soft preference and not a guarantee, and I say so in the [limits](architecture/limits.md).

## The simulator fought back

When I first drove the robot in Gazebo, it went extremely fast, jittered along the path, and drew lines all over the map. It looked like a SLAM problem. The SLAM settings did contribute, but two probes agreed that the main fault was the locomotion: the drive controller had no velocity or acceleration limits, so every command was an instant wheel-speed step and the wheels slipped. The odometry that SLAM relies on was therefore wrong. I also had a cruise speed of 2 m/s that was far too high for this base, and a control loop running on wall-clock time while the rest of the system ran on simulated time. I fixed the locomotion and the clock first, lowered the cruise speed to 0.5 m/s, and had the follower track in the odometry frame so that jumps in the map correction do not reach the wheels.

A ground-truth test then showed the wheel odometry was clean. What remained was SLAM losing its pose. In one run in a nearly empty world it lost it, and in a cluttered world the map held up. I have not established the cause. I added a scan-based safety stop and an optional filter for the robot's pose, and I left the remaining SLAM tuning undone. The reference robot is still an example, and the [reference robot page](architecture/reference-robot.md) says which worlds are worth using.

## The robot parked against a wall

The planner used to refuse any plan whose start was in collision. In the simulator the safety stop could leave the robot a couple of centimetres inside a wall it had only just mapped. Every later replan then failed on its first line, so the robot sat there for good. I first fixed this inside the planner by letting the search tolerate collisions confined to the start footprint, so the robot could drive out.

That fix created a second problem. The path monitor, which re-checks a plan against the newest map, did not know about the exemption and rejected the escape path at its very first pose, which set off an endless stream of replan requests. I patched the monitor to match, and then stepped back: two places now had to agree on a special rule, and I had already found a way to plan through a thin wall with it. I replaced the whole idea. The planner's start gate is hard again, and a separate step moves the robot straight out to free space before planning from a clear start. The rule and its limits are in [the core page](architecture/core.md).

## The collision cache that was not safe

The idea behind the cache is simple. A heading is split into bins (22.5 degrees wide in the shipped settings). For each motion and each bin, I store the cells the footprint touches, then shift that list to the robot's cell at run time. Checking a motion becomes a table lookup.

My first version stored the cells swept from one heading, the lower edge of the bin, starting at the corner of the cell. The search, though, moves a continuous pose, and so does the path monitor that later re-checks the plan. The cells I checked were not the cells the robot would drive over, and the search and the monitor could disagree about whether a path was valid. When I wrote a test that compares the cache against an independent geometric check, more than nine in ten of 20000 random cases had at least one swept cell missing. More bins and shorter motions made each miss smaller but did not make it rarer.

I rebuilt the cache as a guaranteed cover: the stored list contains every cell the footprint can touch from any heading in the bin and any position in the robot's cell. A clear list now proves the motion is clear. The price is that an occupied cell in the list does not prove a collision, because the list is larger than any single sweep. So a motion whose list is blocked is checked exactly before it is expanded. The cover is pinned by a property test, which passed with no misses over 88000 random cases. The construction is in [the swept-cell cover](design/swept-cover.md).

![The cached cover of one motion over one heading bin](media/swept_cover.png)

## The safety stop

The first scan guard stopped the robot when any laser return came within a fixed margin of the footprint, 0.12 m, on the side it was moving towards. Two problems showed up in the simulator. The robot froze beside a wall. And at 0.5 m/s it needed about 0.14 m to stop, more than the margin, so it still ran into walls. A fixed ring is too large beside a wall and too small at cruise speed.

I replaced it with a stop zone that depends on the commanded motion: the area the footprint sweeps during the reaction time and the braking distance, plus a small pad. I also considered and rejected making the planner use a larger footprint to match the guard, because it would have refused goals near walls and blocked narrow gaps the robot fits through. The planner keeps the real footprint, and after repeated stalls the navigator now stops and reports instead of retrying forever. The guard still uses the commanded speed and not the measured one. See [the safety guard](design/safety-guard.md).

![The stop zone](media/stop_zone.png)

## Smaller ones

The navigator measured time with the node clock, and the navigation logic needs times that never go backward. A clock that steps back, which happens in WSL and when a simulator resets, delayed every timer by the size of the step. I now add up only the forward differences of the clock, so a backward step adds nothing and simulated time is still followed. A forward step still fires the pending timers early, and that is in the limits.

Most of the time I lost early on was environment drift, not code: a different compiler version, a conda library shadowing the system one, rendering paths for the simulator. I moved everything into one pinned container image that is the supported build and test environment, and CI uses the same one. See [environment](architecture/environment.md).

## What I would do differently

The collision cache was found wrong by an audit, and a test with an independent check made the size of the problem plain, so that test should come before the cache and not after. The follower should be measured before it is modelled. And a blocked start is its own behaviour, not something to bend the planner around.

What is still not built is listed on the [limits page](architecture/limits.md). The main items are that nothing has run on a physical robot, that nothing bounds how far the robot strays from the planned path, and that a newly seen obstacle reaches the map only after the robot has moved.
