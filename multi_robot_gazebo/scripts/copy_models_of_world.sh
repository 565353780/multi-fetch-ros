#!/bin/bash

# Arguments:
#   1. world file
#   2. directory containing gazebo models
#   3. destination directory for gazebo models
# Example usage
# ./copy_models_of_world.sh worlds/maze.world ~/Projects/OASC-Benchmark/ ./models

RED='\033[0;31m'
NC='\033[0m'

if [[ $# -lt 3 ]]; then
    echo -e "${RED}[ERROR]${NC} Lack of arguments, expected 3 given $#"
    echo "Usage: $0 [world file] [directory with models] [destination directory for models]"
    exit 22
fi

if [[ $1 != *.world || ! -f $1 ]]; then
    echo -e "${RED}[ERROR]${NC} The fisrt argument is not a world file"
    exit 22
fi

if ! [[ -e $3 ]] || ! [[ -d $3 ]]; then
    mkdir "$3"
fi

models=$(sed -nE "/<model name='[s_0-9]+'/s/.*<model name='((s__)?[0-9]+).*'>/\1/p" "$1" | sort -u)
for model_name in $models; do
    mapfile -t model_paths < <(find "$2" -name "$model_name")
    cp -rv "${model_paths[0]}" "$3"
done
