# Current public status

`main` contains the Exploy ReLIC standing integration, bundled graph, simulator
support and offline agent workflow. [README](../README.md) is the entry point.
[Validation](VALIDATION.md) distinguishes the recorded checks from unqualified
hardware timing. Public software state is not a live robot status feed.

The graph is CPU-only; policy 50 Hz, command production 200 Hz. Startup is direct,
initial previous actions are zero, and initial body height comes from measured
standing state. The arm is held stowed. Exploy C++ transport is not integrated.

No robot endpoint, credential, live E-stop state or current physical qualification
record is distributed. Operators must supply their own local configuration and
complete the existing evidence gates. Default activity is offline.

Deployment preparation now has two offline commands: `prepare-deployment` bundles
the candidate, runtime identity and editable review templates; `deployment-check`
checks the frozen artifacts, reviewed hardware inputs, evidence, supplied fresh
preflight and optional local stop status. See [DEPLOYMENT](DEPLOYMENT.md).
