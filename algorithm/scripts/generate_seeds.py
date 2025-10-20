#!/usr/bin/env python3
"""
gen_seeds.py
Generate N random seeds and print them.

Usage:
  # deterministic (reproducible) using a master seed:
  python gen_seeds.py --count 5 --master-seed 123456

  # true-random (non-reproducible):
  python gen_seeds.py --count 5

Output:
  - prints seeds to stdout
  - writes seeds.json and seeds.txt in current folder
"""

import argparse
import json
import secrets
import sys
from typing import List

MAX_INT = 2**31 - 1  # safe 32-bit positive range for many libraries

def generate_deterministic_seeds(master_seed: int, count: int) -> List[int]:
    import numpy as np
    rng = np.random.RandomState(int(master_seed) & 0xFFFFFFFF)
    seeds = [int(s) for s in rng.randint(1, MAX_INT, size=count)]
    return seeds

def generate_random_seeds(count: int) -> List[int]:
    return [secrets.randbelow(MAX_INT - 1) + 1 for _ in range(count)]

# def save_files(seeds: List[int], master_seed: int|None, out_prefix: str = "seeds"):
#     # JSON
#     payload = {
#         "master_seed": int(master_seed) if master_seed is not None else None,
#         "count": len(seeds),
#         "seeds": seeds
#     }
#     with open(f"{out_prefix}.json", "w") as f:
#         json.dump(payload, f, indent=2)

#     # plain text
#     with open(f"{out_prefix}.txt", "w") as f:
#         for s in seeds:
#             f.write(f"{s}\n")

def main():
    p = argparse.ArgumentParser(description="Generate N random seeds (deterministic or cryptographic).")
    p.add_argument("--count", "-n", type=int, default=5, help="Number of seeds to generate (default: 5)")
    p.add_argument("--master-seed", "-m", type=int, default=None,
                   help="If provided, generate seeds deterministically from this master seed")
    p.add_argument("--no-save", action="store_true", help="Don't save seeds to files (only print)")
    args = p.parse_args()

    if args.count <= 0:
        print("Error: count must be positive integer", file=sys.stderr)
        sys.exit(1)

    if args.master_seed is not None:
        seeds = generate_deterministic_seeds(args.master_seed, args.count)
        mode = f"deterministic (master seed={args.master_seed})"
    else:
        seeds = generate_random_seeds(args.count)
        mode = "true-random (secrets)"

    print(f"Generated {len(seeds)} seeds ({mode}):")
    for i, s in enumerate(seeds, start=1):
        print(f"{i:2d}: {s}")

    # if not args.no_save:
    #     save_files(seeds, args.master_seed)
    #     print("\nSaved to: seeds.json, seeds.txt")

if __name__ == "__main__":
    main()
