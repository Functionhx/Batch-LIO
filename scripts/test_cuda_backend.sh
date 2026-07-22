#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cuda_root="${BATCH_LIO_CUDA_ROOT:-/usr/local/cuda-12.8}"
cuda_arch="${BATCH_LIO_CUDA_ARCH:-sm_89}"
build_dir="${repo_root}/.cuda-build"

if [[ ! -x "${cuda_root}/bin/nvcc" ]]; then
  echo "nvcc not found at ${cuda_root}/bin/nvcc" >&2
  echo "Set BATCH_LIO_CUDA_ROOT to a CUDA toolkit compatible with the host compiler." >&2
  exit 2
fi

mkdir -p "${build_dir}"

g++ -std=c++14 -O2 -I"${repo_root}/include" \
  "${repo_root}/test/test_representative_ivox.cpp" \
  -lgtest -lgtest_main -pthread \
  -o "${build_dir}/test_representative_ivox"

"${cuda_root}/bin/nvcc" -std=c++14 -O3 -arch="${cuda_arch}" \
  -I"${repo_root}/include" \
  "${repo_root}/src/cuda/cuda_representative_ivox.cu" \
  "${repo_root}/test/test_cuda_representative_ivox.cpp" \
  -o "${build_dir}/test_cuda_representative_ivox"

"${build_dir}/test_representative_ivox"
"${build_dir}/test_cuda_representative_ivox"

if [[ "${BATCH_LIO_COMPUTE_SANITIZER:-0}" == "1" ]]; then
  "${cuda_root}/bin/compute-sanitizer" --tool memcheck --error-exitcode=99 \
    "${build_dir}/test_cuda_representative_ivox"
fi
