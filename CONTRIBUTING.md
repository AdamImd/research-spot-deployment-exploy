# Contributing

Read AGENTS.md and docs/AGENT_QUICKSTART.md. Install the locked UV environment,
run offline checks, then create a branch from `main`:

```bash
git switch main
git pull --ff-only
git switch -c feature/your-change
```

Use a focused commit and pull request explaining the behavior change, validation,
provenance and remaining limits. Push collaborators can push feature branches;
coordinate changes to runtime contracts and physical gates with the maintainer.
Use GitHub authentication outside the repository. Accept any pending collaborator
invitation before pushing.

Tests and CI must remain offline. Keep raw runs in ignored `runs/` and device
profiles/secrets in ignored `local/`. Publish only reviewed numerical summaries,
not robot identities, network endpoints, payload registrations or credentials.
Preserve source licenses and clearly mark modifications to vendored ReLIC.
