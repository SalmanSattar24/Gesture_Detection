# Employer guide

## Reading path

Read `setup()`, `loop()` and `readMPU6050Data()` in the sketch, then follow the feature-buffer boundary into `run_classifier`. Finally inspect the confidence/motion/static-state decisions and serial output.

## What to discuss

Embedded ML integration, sensor sampling, buffer dimensions, state transitions and observability on a resource-constrained board. The generated inference library is an upstream/tool-generated component, not a model architecture invented in the sketch.

## What is not demonstrated

A threshold of 0.9 is not 90% accuracy. No held-out dataset, fresh firmware compilation, timing trace or cross-user robustness result was measured during this refresh. Reproducing inference requires the correct board, sensor and exported library.
