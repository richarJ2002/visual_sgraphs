# Semantic Linking Rules

How vS-Graphs decides which walls belong to which room, which rooms a
passage connects, and when a room's identity may be resolved against an
already-known room. These rules govern the semantic layer only (rooms,
walls, passages, floors) — they are independent of, and must never depend
on, the free-space occupancy/clustering subsystem (voxblox). Every rule
below is expressed in terms of plane equations, passage aperture geometry,
and room/wall ownership — quantities the semantic layer already owns.

## The core invariant

> When a wall is observed, decide which side of every passage it lies on.
> When inside a room, all walls you see are in the current room. A wall in
> another room is only ever seen through a passage.

Every rule below is an application of this one invariant to a specific
decision: which room a wall belongs to, which room a passage's far side is,
and when two rooms are — or are not — allowed to be treated as the same
room.

## Rule 1 — A passage connects at most two rooms

A `Passage` has exactly one *known* side (the room it was discovered from)
and at most one *prospective/far* side. A passage is never a hub for three
or more rooms. When a passage's far side is genuinely unknown, it stays
unresolved (a placeholder or nothing at all) rather than being guessed at.

## Rule 2 — Crossing a passage's aperture is necessary but not sufficient

Two points (a wall, a room centroid, a camera pose — anything) being on
opposite sides of a passage's plane is not, by itself, proof that they are
each other's direct neighbour through *that* passage. Two further
conditions must both hold:

- **Bounded aperture, not infinite plane.** The crossing point must fall
  within the passage's actual opening — `0.5 * width + margin` horizontally,
  `0.5 * height + margin` vertically — not merely lie on the correct side of
  the passage's infinite supporting plane. Implemented by
  `segmentCrossesPassageOpening()`.
- **No intervening wall.** The straight line between the two points must
  not cross the finite extent of any wall belonging to a *different* room.
  In a corridor with several doors in a row, a distant, unrelated room and
  a genuinely adjacent one can both satisfy the aperture test above purely
  by corridor-alignment coincidence — only an intervening wall proves
  something else sits between them. Implemented by
  `segmentCrossesForeignWall()`, reusing the same finite-wall-segment
  primitives (`buildFiniteWallSegment2d`, `intersectSupportingLines`) used
  elsewhere for wall-topology clash detection.

Any decision that resolves "is room/wall X the far side of passage P" must
apply **both** checks. Neither alone is sufficient.

## Rule 3 — Never test unvalidated position guesses

A prospective room's position at creation is a heuristic
(`passage_centroid + passage_normal * assumed_depth`) — evidence of nothing
except that a passage exists. Rule 2's checks are only meaningful once the
candidate has *real* observed geometry behind it. Concretely: a prospective
room must own at least one real admitted wall before it is tested against
other rooms for far-side resolution. Testing the bare heuristic guess
against Rule 2 can and does produce false matches against unrelated,
distant rooms that happen to sit near the same corridor line.

## Rule 4 — A confirmed room's wall is never re-litigated by a worse guess

Once a wall is owned by a confirmed room, another room's admission attempt
must not steal it — even if that other room's own geometry momentarily
looks favourable. Ownership transfers only through the single admission
chokepoint (`admitWallToRoom`), which checks every passage and never binds
a far-side wall to the near room (Rule 2), and never re-admits a wall
already owned by a distinct confirmed room.

## Rule 5 — Traversal evidence is room-change evidence, not identity evidence

The camera/UAV physically crossing a passage (`Passage::getTraversalEvidence()`)
proves only that a room change happened at that location — nothing about
which room, nor about the passage's own aperture geometry. It is valid
evidence for:

- Allowing a new prospective room to be created even past a resource cap
  (a room change clearly happened; the cap is a resource limit, not an
  identity claim).
- Diagnostics/logging.

It is **not** valid evidence for deciding which confirmed room owns a
contested wall, or for substituting as passage-passability
(`isPassable()`) in any decision that assigns semantic identity. Motion
evidence and wall/room geometry are different kinds of claims and must not
be treated as interchangeable.

## Rule 6 — Every admission/resolution path shares one set of rules

There is more than one place in the code where a wall or room gets
assigned an owner: the primary admission chokepoint, the room-to-room
promotion/retirement search, the creation-time anti-churn check (does a
confirmed far side already exist before creating a placeholder?), and the
wall-ownership conflict resolver. All of them must apply Rules 2–5
identically. A gap in any one of them reopens the failure this document
exists to prevent — history: three of these four locations independently
implemented a bare same-side-of-plane sign test before being brought in
line with Rule 2.

## Known open gap

`admitWallToRoom`'s own far-side backstop tests the *admitting room's own
centroid* as one endpoint of the Rule-2 check. That degenerates — silently
returns "no crossing" — when the room's centroid sits within
`minimumSideDistance_m` of its own passage's plane, which is common for a
sparsely-observed room built from just the one wall bordering that
passage. Suspected but not yet confirmed as the cause of at least one
observed far-wall misassignment. Not yet fixed.
