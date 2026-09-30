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

## Key design decisions

- **ADRs are immutable once accepted.** A reversed decision gets a new ADR that supersedes
  the old one, so the reasoning trail survives. Silently editing an ADR destroys the record
  of why the project ever thought otherwise.
- **Theory is written before the controller it justifies**, because a derivation that cannot
  be written down is a controller that cannot be defended.
- **Learning notes are for the owner, not for reviewers.** If a question in a learning note
  cannot be answered without rereading the code, the code needs better comments.

## Known limitations

- `theory/`, `bringup/`, and `learning/` are empty: no controller has been designed, no
  hardware powered, and no owner-reviewed module implemented.
