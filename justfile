# Racing Playground – task runner
# Install just: https://just.systems  (or: cargo install just)
# All commands here are thin wrappers – the real logic lives in scripts/.

WORKSPACE  := "/home/ubuntu/ws/src/racing_playground"
ROS_SETUP  := "/opt/ros/humble/setup.bash"
WS_INSTALL := "/home/ubuntu/ws/install/setup.bash"

# ── default: list all recipes ────────────────────────────────────────────────
default:
    @just --list

# ── add ───────────────────────────────────────────────────────────────────────
# Scaffold a complete map directory.
# Usage: just add map --name <NAME> --path <FILE.png> [--overwrite]
add subcommand *args:
    #!/usr/bin/env bash
    set -eo pipefail
    case "{{subcommand}}" in
      map)
        python3 "{{WORKSPACE}}/scripts/add_map.py" {{args}}
        ;;
      *)
        echo "Unknown subcommand: '{{subcommand}}'   (available: map)"
        exit 1
        ;;
    esac

# ── sim ───────────────────────────────────────────────────────────────────────
# Launch a simulation.
# Usage: just sim base --map <MAP_NAME> [--racecar-version SIM]
sim subcommand *args:
    #!/usr/bin/env bash
    set -eo pipefail
    case "{{subcommand}}" in
      base)
        eval set -- {{args}}
        "{{WORKSPACE}}/scripts/sim.sh" "$@"
        ;;
      *)
        echo "Unknown subcommand: '{{subcommand}}'   (available: base)"
        exit 1
        ;;
    esac

# ── build ─────────────────────────────────────────────────────────────────────
# Build the ROS 2 workspace.
# NOTE: sourcing install/setup.bash runs in a subshell; run it manually after.
build:
    "{{WORKSPACE}}/scripts/build.sh"
