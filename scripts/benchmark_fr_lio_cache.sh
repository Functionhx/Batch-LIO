#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${repo_root}/.cuda-build"
mkdir -p "${build_dir}"

g++ -std=c++14 -O3 -DNDEBUG -I"${repo_root}/include" \
  "${repo_root}/bench/benchmark_representative_ivox.cpp" \
  -o "${build_dir}/benchmark_representative_ivox"

"${build_dir}/benchmark_representative_ivox" "${1:-120000}"
