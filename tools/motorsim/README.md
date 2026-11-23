# motorSim oracle

`ioc.yaml` configures an 8-axis IOC running the genuine EPICS `motorRecord` on
the `motorSim` asyn driver. It is the reference implementation that
`tools/motor_oracle.py` compares this adapter against.

See [docs/validation.md](../../docs/validation.md) for how to run it and what
the comparison does and does not check.

Image: `ghcr.io/epics-containers/ioc-motorsim-runtime:R1-3ec6`
(EPICS base 7.0.11, motor R7-4, motorMotorSim R1-3, asyn R4-46).
There is no `latest` tag; pin a release. The `R1-3ec*` tags are multi-arch.
