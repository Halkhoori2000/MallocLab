# MallocLab — Custom Dynamic Memory Allocator

**[Live Showcase →](https://halkhoori2000.github.io/MallocLab/)**

A from-scratch implementation of `malloc`, `free`, `realloc`, and `calloc` in C, achieving **20,092 Kops/s** throughput with only **8 bytes of overhead** per allocation.

---

## Overview

This project replaces the standard library allocator with a hand-written implementation backed by segregated free lists. The allocator manages a heap obtained via `sbrk`, organises free blocks into 8 size classes for O(1) class lookup, and uses a combination of boundary tags and a prev-allocated bit to perform immediate coalescing without storing a footer in allocated blocks.

The implementation handles all four standard allocation functions — `malloc`, `free`, `realloc`, `calloc` — and passes the full mdriver test suite.

## How It Works

The heap is divided into variable-size blocks. Each block carries an 8-byte header encoding its size and three flag bits: whether it is allocated, whether the preceding block is allocated, and whether it is a 16-byte small free block. Free blocks additionally carry a footer (for coalescing) and two 8-byte pointers linking them into a doubly-linked free list for their size class.

**Small block optimisation**: 16-byte free blocks are special-cased — their header and footer words double as `prev` and `next` free-list pointers, so no separate link fields are needed, keeping minimum block size at 16 bytes.

**Prev-allocated bit**: allocated blocks have no footer. Instead, each block's header stores whether the preceding block is free, so the coalescer can find the predecessor without a footer round-trip.

On `malloc`, the allocator finds the best-fit block in the matching size class (or the next non-empty larger class), splits if the remainder is large enough, and extends the heap with `sbrk` only when no free block fits. On `free`, the block is immediately coalesced with adjacent free neighbours and inserted at the head of the appropriate free list.

## Use Cases

- Understanding how heap allocators work at the hardware/OS boundary
- Baseline for experimenting with allocation strategies (first-fit, best-fit, LIFO vs address-ordered lists)
- Reference implementation for systems programming courses covering memory management
- Performance comparison against `libc` malloc on allocation-heavy workloads

## Challenges

- **Eliminating the allocated-block footer**: Implementing the prev-allocated bit correctly required updating the neighbour's header on every alloc/free to keep the bit in sync, including edge cases during coalescing and heap extension.
- **Small block special-case**: 16-byte blocks cannot hold both link pointers and a separate footer; overlapping the header/footer words with the list pointers required careful pointer arithmetic and separate code paths throughout `malloc`, `free`, and coalesce.
- **Best-fit within a size class**: Scanning only the matching size class for best-fit (rather than the whole heap) bounds search cost while still reducing fragmentation compared to first-fit.
- **Heap alignment**: Every allocated block must be 16-byte aligned; the allocator adjusts requested sizes up to the next multiple of 16 and accounts for header padding when splitting blocks.
- **Coalescing correctness under all four cases**: Handling all four neighbour combinations (prev-alloc/free × next-alloc/free) without corrupting free-list pointers required thorough testing with the mdriver trace suite.

## Tech Stack

| Component | Detail |
|---|---|
| Language | C (C99) |
| Allocation strategy | Segregated free lists, best-fit |
| Size classes | 8 (starting at 16 bytes) |
| Block overhead | 8 bytes (header only for allocated blocks) |
| Alignment | 16-byte |
| Heap extension | `sbrk` via `mem_sbrk` wrapper |
| Test driver | `mdriver` (provided harness) |

## Project Structure

```
src/
├── mm.c          # allocator implementation (malloc/free/realloc/calloc)
├── mm.h          # public interface
├── memlib.c/h    # heap model (sbrk wrapper)
├── mdriver.c     # test harness
├── Makefile      # build
└── throughputs.txt
```

## Build & Run

```bash
cd src
make
./mdriver -v        # run all traces, verbose
./mdriver -t traces # run a specific trace directory
```

Requires GCC and standard POSIX headers. Tested on Linux x86-64.
