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
