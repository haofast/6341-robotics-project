#!/bin/zsh
# Run from the LeRobot environment with the MoveIt planning launch active.
set -eu

if (( $# < 2 )); then
    print -u2 "Usage: zsh vision-service/run_moveit_grasp.zsh ROW COLUMN [SPEED_DEG_PER_SEC] [--drop ROW COLUMN] [--lift-mm MM]"
    exit 2
fi

row=$1
column=$2
shift 2
speed=30
if (( $# > 0 )) && [[ "$1" != --* ]]; then
    speed=$1
    shift
fi
motion_args=(--lift-mm 60 "$@")
if ! python -c 'import sys; sys.exit(not 0 < float(sys.argv[1]) <= 30)' "$speed" 2>/dev/null; then
    print -u2 "Speed must be greater than 0 and at most 30 degrees per second."
    exit 2
fi

script_dir=${0:A:h}
setup_dir=${SO101_SETUP_DIR:-"$script_dir/setup-data/3x6"}
workspace_args=(--workspace "$setup_dir/workspace.json" --pickup-poses "$setup_dir/pickup_poses.json")
run_dir=$(mktemp -d /tmp/so101-grasp.XXXXXX)
print "Run files and log saved in $run_dir"
exec > >(tee "$run_dir/run.log") 2>&1

python "$script_dir/moveit_pick.py" "$row" "$column" \
    --prepare --current-start "${motion_args[@]}" "${workspace_args[@]}" --targets "$run_dir/targets.json"

(
    unsetopt nounset
    source /opt/homebrew/opt/ros2-jazzy/setup.zsh
    /Library/Frameworks/Python.framework/Versions/3.11/bin/python3 \
        "$script_dir/moveit_pick.py" "$row" "$column" --plan "${workspace_args[@]}" \
        --targets "$run_dir/targets.json" --export-grasp "$run_dir/grasp.json"
)

python "$script_dir/run_moveit_empty.py" "$run_dir/grasp.json" --grasp --speed "$speed" --execute
