# Repository rules
- Treat the original canonical YAML profiles as the source of operational parameters.
- Preserve existing settings; extend canonical files rather than replacing them.
- Do not duplicate YAML parameter values or parsers in launch files.
- Change C++ only when required behavior is absent; preserve FAST-LIVO algorithms.
- Test LO, LIO, and LIVO mode selection and ROS 2 Humble behavior.
- Preserve stable branches; implement Codex work only on isolated experimental branches.
- Agents must not edit or commit code on Jetson systems.
- Keep tests reproducible and changes suitable for atomic commits.
