# Timing

The Exploy runtime currently uses CPU inference. The recorded 5 ms release gate failed once. See [measured results](VALIDATION.md). The benchmark tool accepts a v3 CPU manifest and an original `--reference-checkpoint`; no robot packets are sent. CUDA inference for this graph is rejected explicitly.
