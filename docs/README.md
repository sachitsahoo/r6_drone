# docs — architecture, decisions, theory, bring-up, learning

## What it does

The written half of the project. The owner must be able to understand, explain, and defend
every design decision here in interviews and research writeups, so documentation is part of
every task rather than cleanup afterward.

## Layout

| Directory | Contents |
|---|---|
| `decisions/` | Architecture Decision Records, `NNNN-short-title.md`. One per real choice between reasonable alternatives: context, options considered, decision, consequences. |
| `theory/` | Control and estimation derivations: equations with every variable defined, block diagrams, references. Code comments link to the relevant section rather than restating the math. |
| `bringup/` | Hardware bring-up procedures and the measurements they produced. |
| `learning/` | For each owner-reviewed module, a plain-language walkthrough plus 5-10 "check your understanding" questions with answers. |

`mechanical-requirements.md` sits at this level rather than in a subdirectory: it is neither a decision nor a derivation, but the set of design inputs the ADRs and the theory note impose on the CAD.

## Key design decisions

- **ADRs are immutable once accepted.** A reversed decision gets a new ADR that supersedes
  the old one, so the reasoning trail survives. Silently editing an ADR destroys the record
  of why the project ever thought otherwise.
- **How a later decision reaches an accepted ADR.** The new ADR says what it amends or
  supersedes. The old one gets only two things: a status line listing what has been
  overtaken and by which ADR, and dated callouts (`> **Decided since:** ...`,
  `*(Amended by ADR NNNN: ...)*`) next to each affected passage. Its original text stays in
  place. A reader landing on any section can tell whether it is still current, without
  losing what was believed before.
- **Proposed ADRs are living drafts.** Until accepted, they are edited in place to stay
  correct — a proposal waiting for approval should never be stale.
- **Theory is written before the controller it justifies**, because a derivation that cannot
  be written down is a controller that cannot be defended.
- **Learning notes are for the owner, not for reviewers.** If a question in a learning note
  cannot be answered without rereading the code, the code needs better comments.

## Known limitations

- `theory/` has the pitch-axis budget only; no controller has been designed yet.
- `bringup/` has the UART link plan and the component measurement checklist, but no
  hardware has been powered.
- `learning/` has the protocol and HAL walkthroughs.
