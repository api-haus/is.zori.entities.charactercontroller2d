# Code style (binding)

Calibration: internal uniformity and subtractive discipline — no comment deletable without information loss, no defensive check the type system rules out, no doc restating a member name.

## Naming
- Fields: `m_PascalCase` instance, `s_PascalCase` static, `k_PascalCase` const. Public members PascalCase (Framework Design Guidelines); parameters/locals camelCase.
- Existing public field casing is API — changing it is an API break, not a style fix.
- New types are named for what they own. `Helper/Manager/Utils/Handler/Processor/Service/Wrapper` are banned for new types.

## Structure
- Allman braces, 4-space indent, explicit visibility first, `readonly` wherever applicable, expression bodies for true one-liners only.
- File length follows concept boundaries, not a line budget; split large API families by feature via `partial`, never by line count.
- Overload families in strict parameter-progression order with identical body shapes.
- No `#region`. No legacy shims, no dead parameters, no "kept for compatibility" residue.

## Comments
- Every public symbol: a one-line `<summary>` with information not derivable from the signature. Internals undocumented unless the invariant is load-bearing.
- Inline `//` states *why*, never *what*: invariants, compat/allocation constraints, algorithm citations (URL or paper), honest `TODO:` with the actual defect.
- One canonical home per fact — `Documentation~/` holds contracts; code carries one-line pointers, never restatements.
- The canonical statement is itself brief (fewest sentences, normally <8 lines; longer → move to `Documentation~/` + pointer); it never enumerates its consumers; never mirror a fact "exactly like X" restates. A comment block much longer than the code it guards is suspect — keep only sentences a maintainer of this file needs to avoid a mistake.
- Comments describe current state, never history: no "X removed", no quoted deleted code, no tuning-session numbers, no orchestration step references.
- Density band 5–20%. Never narrate the next line. No marketing vocabulary (robust/comprehensive/seamless/gracefully), no emoji.

## Unity
- Never rename a serialized field without `[FormerlySerializedAs]`; never delete Unity message methods or `[SerializeField]` fields as "unused".
- Runtime math: `using static Unity.Mathematics.math;`, `float3`/`float4x4` over `Vector3`/`Matrix4x4`.
- HLSL: `#pragma once`.
- Formatter: `csharpier format .`; a trailing `//` preserves a hand-authored break — only where the break carries meaning, never as a reflex.

## Forbidden in new/edited code
Blanket `catch (Exception)`; null checks construction order rules out; tautological `<summary>`s; redundant `else` after `return`; idiom inconsistency with the host file; boilerplate enumeration of what a computation expresses.

## Port fidelity (binding — read before refactoring the Runtime)

The Runtime is a method-for-method 2D port of Unity's `com.unity.charactercontroller` (originally "Rival" by Philippe St-Amand, now an official Unity DOTS package). Each long solve method — `SolveOverlaps`, `MoveWithCollisions`, `CheckForSteppingUpHit`, `ProcessCharacterHitDynamics`, `GroundDetection`, and the rest — mirrors one upstream method by name and shape. A maintainer verifies this port by diffing each method against its upstream counterpart, so the method boundaries and the inline `// … (REF/<file>:<line>)` provenance citations ARE the verification surface, not noise. A refactor that decomposes a port-faithful method, or deletes the REF citations as "cleanup", breaks upstream-diffability and is regression, not improvement. The longest-methods list and the high comment density are port-fidelity features the generic smell catalogue misreads.

### Citations the reader can reach vs the porting process's own bookkeeping

Port fidelity protects only the citations a reader of the shipped package can resolve: the `// … (REF/<file>:<line>)` upstream pointers and the upstream commit SHA both name external source a maintainer can fetch. It does not protect references to the porting *process*. The chunk labels (`C1`–`C6`, "chunk C4"), the design-decision tags (`D1`–`D8`, "design D3", "motion-drive D6"), and the design-document section pointers ("design §6", "design section 5") resolve only against this package's orchestration journal, which does not ship and is not canonical, so to a maintainer of the shipped file they are dangling pointers — the "no orchestration step references" case of the comment rule above, not provenance. Strip each to the self-contained fact it stood for: "the C3 contract" becomes the named edges (`[UpdateAfter(Store…)]` and `[UpdateBefore(Deferred…)]`), "runs the C4a core solve chain" becomes "runs the core solve chain", and "the same store-then-read pattern C3 already uses" names the system it referred to (`StoreKinematicCharacterBodyPropertiesSystem2D`). The rationale a tag introduced stays; only the pointer into the non-shipping document goes.

### Resolving the REF citations

Every `REF/<file>:<line>` citation resolves against the upstream `com.unity.charactercontroller` source — the upstream is not vendored in this repo, so the `REF/` paths are external by design. The upstream package version the line numbers pin against is not recorded in this repo; resolving a citation against a specific upstream revision requires fetching that package and accounting for line drift between revisions.

The bare `c719d90` commit SHA in `Runtime/KinematicCharacterUtilities2D.cs` (the `ReconstructOverlap` depth-projection comment) is an upstream commit, not a commit in this repo's history — it is unreachable here and names a fix in the upstream lineage, the same external-only reachability as the `REF/` paths.
