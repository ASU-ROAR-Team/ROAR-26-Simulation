# Simulation repo changes

Changes made to the external simulation repo
(`${ROAR_WS}`), extracted out of `CHANGELOG.md`
(which covers `nav-stack_2026` itself) since that repo is separate from
this one. Context/reasoning for *why* these exist lives in
`CHANGELOG.md` at this repo's root — this file just tracks what actually
changed over there.

## Currently active

**File:** `src/roar_simulation/config/diff_controller.yaml`

Added `enable_odom_tf: false` to `diff_drive_controller`'s parameters:

```yaml
    use_stamped_vel: false  # Allows standard Twist messages instead of TwistStamped

    # odom->base_link TF is instead broadcast by diff_drive_odom_correction_node
    # (roar_localization), which corrects a 90deg wheel-axis mislabeling before
    # publishing. See nav-stack_2026/CHANGELOG.md. To revert: delete this line
    # (or set true) and stop launching diff_drive_odom_correction_node.
    enable_odom_tf: false
```

**Why:** `ieskf_node` (in `nav-stack_2026`) now broadcasts the real
`odom → base_link` TF instead of `diff_drive_controller`. Without this,
`diff_drive_controller` would broadcast its own (wrong) `odom → base_link`
at the same time, and the two would fight over the same TF edge.

**To revert:** delete the `enable_odom_tf: false` line (or set it to
`true`) — but only if `ieskf_node`'s `publish_tf` is also set back to
`False` in `nav-stack_2026`'s `roar_nav2_only.launch.py`; otherwise
nothing will broadcast `odom → base_link` at all.

The comment's mention of `diff_drive_odom_correction_node` is slightly
stale — that node no longer owns any TF either; `ieskf_node` does. Left
as-is since it doesn't affect correctness (still describes why
`diff_drive_controller`'s own broadcast is off).
