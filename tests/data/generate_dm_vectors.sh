#!/bin/sh
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Yaolin Dong
# The public tree contains the fixed oracle for comparison only. Regeneration
# requires the original pre-refactor driver, supplied separately by a maintainer.
set -eu
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 REFERENCE_TREE (original pre-refactor Core/Src and Core/Inc required)" >&2
    exit 2
fi
reference=$(cd "$1" && pwd)
cd "$(dirname "$0")/../.."
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
cp "$reference/Core/Src/dm_motor.c" "$reference/Core/Inc/dm_motor.h" "$tmp/"
cc -std=c11 -O2 -ffp-contract=off -DGENERATE_VECTORS '-D__weak=__attribute__((weak))' \
    -I"$tmp" -Itests/mocks/protocol_oracle "$tmp/dm_motor.c" \
    tests/test_dm_protocol_equivalence.c -lm -o "$tmp/generate"
"$tmp/generate" > "$tmp/records"
gzip -9nc "$tmp/records" > "$tmp/vectors.gz"
cp "$tmp/vectors.gz" tests/data/dm_protocol_vectors.bin.gz
