#!/usr/bin/env bash

# We need to get into the .runfiles directory in order for sdformat to find root.sdf. See #7874.
cd bazel-bin/examples/contact_model/mbp/rigid_mug_gripper_mbp.runfiles/drake

./examples/contact_model/mbp/rigid_mug_gripper_mbp \
                  --gripper_force=1e-8 \
                  --integration_scheme=runge_kutta2 --time_step=1e-8 --accuracy=1e-3 \
                  --v_stiction_tolerance=1e-3 \
                  --ring_static_friction=0.0 --ring_dynamic_friction=0.0 \
                  --ring_samples=4 --pad_depth=3e-3 --ring_orient=0 \
                  --px=0 --py=0 --pz=0 --rx=0 --ry=0 --rz=0 \
                  --finger_width=-0.08811
cd -
