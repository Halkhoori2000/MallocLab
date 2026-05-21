/*
 * mm.c
 *
 * Name: [HAMDAN ALKHOORI]
*
 * This is a malloc implementation where I used segregated free lists to
 * approximate a best fit rule. To keep the speed of throughput high, I
 * used a previous allocated flag to quickly check if the block can be
 * coalesced with previous block while reducing the additional memory
 * rqeuired for each block. In fact, we only need to add 8 bytes to
 * store the header information of the block. We also allow a special
 * free block called small block which is used to represent 16 bytes
 * free block. Instead of saving the small block size, the third bit
 * is used to flag a block as small. It allows the user of the header
 * and footer fields to store the previous and next free block links.
 *
 */
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>

#include "mm.h"
#include "memlib.h"

/*
 * If you want to enable your debugging output and heap checker code,
 * uncomment the following line. Be sure not to have debugging enabled
 * in your final submission.
 */
//#define DEBUG

#ifdef DEBUG
// When debugging is enabled, the underlying functions get called
#define dbg_printf(...) printf(__VA_ARGS__)
#define dbg_assert(...) assert(__VA_ARGS__)
#else
// When debugging is disabled, no code gets generated
#define dbg_printf(...)
#define dbg_assert(...)
#endif // DEBUG

// do not change the following!
#ifdef DRIVER
// create aliases for driver tests
#define malloc mm_malloc
#define free mm_free
#define realloc mm_realloc
#define calloc mm_calloc
#define memset mm_memset
#define memcpy mm_memcpy
#endif // DRIVER

// Define types
typedef uint64_t header_t;

// Useful constants in the program
#define ALIGNMENT 16
#define BIT_ALLOCATED 0x1
#define BIT_PREVIOUS_ALLOCATED 0x2
#define BIT_SMALL_FREE_BLOCK 0x4
#define INITIAL_PAGE_SIZE 1024
#define PAGE_SIZE 16
#define FLAG_MASK 7
#define SIZE_MASK (~FLAG_MASK)
#define SMALL_FREE_BLOCK_SIZE 16
#define NUM_FREE_LISTS 8
#define START_FREE_LIST_SIZE 16
#define START_BLOCK_SIZE sizeof(header_t)
#define END_BLOCK_SIZE ((size_t)0)

/**
 * Design Comments 
 * 
 * This dynamic memory allocator used three kinds of blocks: allocated block,
 * small free block, and free block. The structures of each type of
 * block are shown below.
 * 
 * Figure 1. Allocated Block: 
 * ---------------------------------------------
 * | HEADER                                    |
 * ---------------------------------------------
 * | PAYLOAD                                   |
 * |                                           |
 * ---------------------------------------------
 *
 * Figure 5. Small Free Block:
 * ---------------------------------------------
 * | HEADER/PREVIOUS LINK                      |
 * ---------------------------------------------
 * | FOOTER/NEXT LINK                          |
 * ---------------------------------------------
 * Figure 4. Free Block:
 * ---------------------------------------------
 * | HEADER                                    |
 * ---------------------------------------------
 * | NEXT LINK                                 |
 * ---------------------------------------------
 * | PREVIOUS LINK                             |
 * ---------------------------------------------
 * | UNUSED SPACE                              |
 * |                                           |
 * ---------------------------------------------
 * | FOOTER                                    |
 * ---------------------------------------------
 * 
 * This data structure allows us to maximize the space utilization of the dynamic
 * allocator since we only need to add eight (8) bytes to the required size. 
 * 
 * We also used segregated free list to approximate a best fit search in the free 
 * list. Although doubly-linked list requires more memory per node, we used it since
 * it allows quick insertion and deletion of free blocks in/out of the free list.
 * 
 * We also used footer to be able to find previous adjacent block in the heap. This
 * feature is useful for coalescing blocks after an allocated block is freed. 
 * 
 * To distinguish between different types of blocks we use the following header structure:
 * Figure 2.
 * ---------------------------------------------
 * | SIZE                     | SFF | PAF | AF |
 * ---------------------------------------------
 * Size field contains the block size except for small free block. 
 * SFF is the small free block flag. When set, the block is a small free block.
 * PAF is the previous allocated block flag. When set, the previous adjacent block is allocated.
 * AF is the allocated flag. When set, the block is allocated.
 * 
 * The number of segregated free lists are maximized to split the free blocks into more group.
 * More groups means more fine grain search of free blocks. The size of each segregated free list
 * is double the previous to give a logarithmic search of new free blocks which is faster than a
 * linear search. 
 */

/**
 * @brief Figure 6. This is union block that has three types of block structure.
 * It allows easy extracttion of the fields depending on the type of
 * the block. For allocated block, it contains the header and the
 * payload. For the free block, it contains the header block and followed
 * by next and prev block. The footer field is not shown as the size
 * of the struct varies. For the small free block, it contains only the
 * previous and next links.
 * 
 * 
 */
typedef union block
{
    struct
    {
        header_t header;
        char payload[];
    } allocated_blk;

    struct
    {
        header_t header;
        union block *next;
        union block *prev;
    } free_blk;

    struct
    {
        union block *prev;
        union block *next;
    } small_free_blk;

} block_t;

// Figure 7.
typedef struct
{
    union block *prev;
    union block *next;
} small_block_t;

// We use a circular free list that allows us to easily
// insert and remove to the beginning or the end of the free list
static small_block_t free_lists[NUM_FREE_LISTS];

// rounds up to the nearest multiple of ALIGNMENT
static size_t align(size_t x)
{
    // Compute the ceiling to get the next largest number divisible by ALIGNMENT
    return ALIGNMENT * ((x + ALIGNMENT - 1) / ALIGNMENT);
}

/**
 * @brief Align the input size based on the prevailing page size
 *
 * @param size number of bytes
 * @return size_t aligned size
 */
static size_t align_page(size_t size)
{
    // Do the same with align but use the PAGE_SIZE as divisor
    return PAGE_SIZE * ((size + PAGE_SIZE - 1) / PAGE_SIZE);
}

// Helper functions

/**
 * @brief Figure 13. Set the properties of the block on header and footer (if available). The properties
 * are set at the same time to prevent hidden bugs caused by setting different parameters individually.
 *
 * @param block pointer to block
 * @param size block size
 * @param prev_allocated previous allocated flag
 * @param allocated allocated flag
 * @param prev_free previous free block link (unused if block is allocated)
 * @param next_free next free block link (unused if block is allocated)
 */
static void block_set_props(block_t *block, size_t size, bool prev_allocated, bool allocated, block_t *prev_free, block_t *next_free)
{
    dbg_assert(block);

    if (allocated)
    {
        // For allocated block, only update the header
        block->allocated_blk.header = size + (prev_allocated ? BIT_PREVIOUS_ALLOCATED : 0) + BIT_ALLOCATED;
    }
    else if (size == SMALL_FREE_BLOCK_SIZE)
    {
        // For small free block, input the next and previous links and the flags
        header_t header = (prev_allocated ? BIT_PREVIOUS_ALLOCATED : 0) + BIT_SMALL_FREE_BLOCK;
        block->small_free_blk.prev = (block_t *)((char *)prev_free + header);
        block->small_free_blk.next = (block_t *)((char *)next_free + header);
    }
    else
    {
        // For free block, update both the header and footer fields
        header_t header = size + (prev_allocated ? BIT_PREVIOUS_ALLOCATED : 0);
        block->free_blk.header = header;
        block->free_blk.next = next_free;
        block->free_blk.prev = prev_free;
        header_t *footer_ptr = (header_t *)((char *)block + size - sizeof(header_t));
        *footer_ptr = header;
    }
}

/**
 * @brief Get the size of the block
 *
 * @param block pointer to the block
 * @return size_t extracte size of the block
 */
static size_t block_size(block_t *block)
{
    dbg_assert(block);

    header_t header = *(header_t *)block;

    // For small free block return the 16 bytes
    if (header & BIT_SMALL_FREE_BLOCK)
    {
        return SMALL_FREE_BLOCK_SIZE;
    }
    // Extract the size from the header field
    else
    {
        return header & SIZE_MASK;
    }
}

/**
 * @brief Get the allocated flag of the block
 *
 * @param block pointer to a block
 * @return true if allocated
 * @return false otherwise
 */
static bool block_allocated(block_t *block)
{
    dbg_assert(block);

    // Extract the header field
    header_t header = *(header_t *)block;

    // Check if the AF flag is set
    return (header & BIT_ALLOCATED) != 0;
}

/**
 * @brief Get the previous allocated flag of the block
 *
 * @param block pointer to a block
 * @return true if allocated
 * @return false otherwise
 */
static bool block_prev_allocated(block_t *block)
{
    dbg_assert(block);

    // Extract the header field
    header_t header = *(header_t *)block;

    // Check if the PAF flag is set
    return (header & BIT_PREVIOUS_ALLOCATED) != 0;
}

/**
 * @brief Figure 15. Set the next free link of the free block
 *
 * @param block pointer to a block
 * @param free_block the next free block
 */
static void free_block_set_next(block_t *block, block_t *free_block)
{
    dbg_assert(free_block);
    dbg_assert(!block_allocated(free_block));
    dbg_assert(block);
    dbg_assert(!block_allocated(block));

    // Extract the header field
    header_t header = *(header_t *)block;

    // For small free blcok, extract the flag mask and apply it into the new free block pointer
    if (header & BIT_SMALL_FREE_BLOCK)
    {
        block->small_free_blk.next = (block_t *)((char *)free_block + ((header_t)block->small_free_blk.next & FLAG_MASK));
    }
    else
    {
        // Set the next link field
        block->free_blk.next = free_block;
    }
}

/**
 * @brief Figure 14. Set the previous free link of the free block
 *
 * @param block pointer to a block
 * @param free_block the previous free block
 */
static void free_block_set_prev(block_t *block, block_t *free_block)
{
    dbg_assert(free_block);
    dbg_assert(!block_allocated(free_block));
    dbg_assert(block);
    dbg_assert(!block_allocated(block));

    // Extract the header field
    header_t header = *(header_t *)block;

    // For small free block, extract the flag mask and apply it into the new free block pointer
    if (header & BIT_SMALL_FREE_BLOCK)
    {
        // For small blocks, keep the header flags
        block->small_free_blk.prev = (block_t *)((char *)free_block + ((header_t)block->small_free_blk.prev & FLAG_MASK));
    }
    else
    {
        // Set the previous link field
        block->free_blk.prev = free_block;
    }
}

/**
 * @brief Get the next free block
 *
 * @param free_block pointer to a free block
 * @return block_t* next link
 */
static block_t *free_block_get_next(block_t *free_block)
{
    dbg_assert(free_block);
    dbg_assert(!block_allocated(free_block));

    // Extract the header field
    header_t header = *(header_t *)free_block;

    // For samll free block, extract the pointer using the SIZE MASk
    if (header & BIT_SMALL_FREE_BLOCK)
    {
        // For small free blocks, filter out the header flags
        return (block_t *)((header_t)free_block->small_free_blk.next & SIZE_MASK);
    }
    else
    {
        // Extract the next link field
        return free_block->free_blk.next;
    }
}

/**
 * @brief Get the previous free block
 *
 * @param free_block pointer to a free block
 * @return block_t* previous link
 */
static block_t *free_block_get_prev(block_t *free_block)
{
    dbg_assert(free_block);
    dbg_assert(!block_allocated(free_block));

    // Extract the header field
    header_t header = *(header_t *)free_block;

    // For small free block, extract the pointer using the SIZE MASK
    if (header & BIT_SMALL_FREE_BLOCK)
    {
        // For small free blocks, filter out the header flags
        return (block_t *)((header_t)free_block->small_free_blk.prev & SIZE_MASK);
    }
    else
    {
        // Extract the previous link field
        return free_block->free_blk.prev;
    }
}

/**
 * @brief Find the free list index according to the block size
 *
 * @param size block size
 * @return int index to the free list
 */
static int find_free_list_index(size_t size)
{
    // Loop through each free list. Starting from the smallest size up
    // to the largest size
    for (int i = 0; i < NUM_FREE_LISTS - 1; i++)
    {
        // Compute the maximum free block in side the free list
        size_t list_max_size = START_FREE_LIST_SIZE << i;

        // If the size is enough, return the index 
        if (list_max_size >= size)
        {
            return i;
        }
    }

    // For very large blocks, use the last free list
    return NUM_FREE_LISTS - 1;
}

/**
 * @brief Find the free list according to the block size
 *
 * @param size block size
 * @return block_t* pointer to the free list
 */
static block_t *find_free_list(size_t size)
{
    // Use the index and convert the pointer
    return (block_t *)&free_lists[find_free_list_index(size)];
}

/**
 * @brief Figure 16. Insert the free block to the free lists
 *
 * @param free_block pointer to the free list
 */
static void free_blocks_insert(block_t *free_block)
{
    dbg_assert(free_block);
    dbg_assert(!block_allocated(free_block));

    // Find the list to insert the free block
    block_t *list = find_free_list(block_size(free_block));

    // Insert the free block at the end of the list
    block_t *tail = free_block_get_prev(list);

    free_block_set_next(tail, free_block);
    free_block_set_next(free_block, list);

    free_block_set_prev(free_block, tail);
    free_block_set_prev(list, free_block);
}

/**
 * @brief Remove the free block from the free list
 *
 * @param free_block
 */
static void free_blocks_remove(block_t *free_block)
{
    dbg_assert(free_block);
    dbg_assert(!block_allocated(free_block));

    // Get the previous and next free blocks
    block_t *prev = free_block_get_prev(free_block);
    block_t *next = free_block_get_next(free_block);

    // Link them together to skip the free block
    free_block_set_next(prev, next);
    free_block_set_prev(next, prev);

    // Clear links of the free block
    block_set_props(free_block,
                    block_size(free_block),
                    block_prev_allocated(free_block),
                    block_allocated(free_block), NULL, NULL);
}

/*
 * mm_init: returns false on error, true on success.
 * Figure 8. 
 */
bool mm_init(void)
{
    // Allocate initial block
    char *initial_block = mm_sbrk(INITIAL_PAGE_SIZE);
    if (!initial_block)
    {
        return false;
    }

    // Figure 9. Initialize the free lists by linking the free list to itself
    for (int i = 0; i < NUM_FREE_LISTS; i++)
    {
        block_set_props((block_t *)&free_lists[i], sizeof(small_block_t), false, false, NULL, NULL);
        free_block_set_prev((block_t *)&free_lists[i], (block_t *)&free_lists[i]);
        free_block_set_next((block_t *)&free_lists[i], (block_t *)&free_lists[i]);
    }

    // Figure 10. Create the start and end block
    // Set the start block as an allocated block with the its previous block allocated
    block_set_props((block_t *)initial_block, START_BLOCK_SIZE, true, true, NULL, NULL);

    // Figure 11. Set the end block as an allocated block with zero size to prevent the next block
    // from going outside the heap
    block_t *end_block = (block_t *)(initial_block + INITIAL_PAGE_SIZE - sizeof(header_t));
    block_set_props(end_block, END_BLOCK_SIZE, false, true, NULL, NULL);

    // Figure 12. Create first free block and insert into the list
    block_t *free_block = (block_t *)(initial_block + sizeof(header_t));
    block_set_props(free_block, INITIAL_PAGE_SIZE - 2 * sizeof(header_t), true, false, NULL, NULL);

    // Add the free block to the free lists
    free_blocks_insert(free_block);

    return true;
}

/**
 * @brief Find a block given the required size of the block. It searches the smallest free list that
 * could fit the block size and proceeds to check the other free lists.
 *
 * @param size block size
 * @return block_t* pointer of the free block
 */
static block_t *block_find(size_t size)
{
    // Find the free list
    for (int i = find_free_list_index(size); i < NUM_FREE_LISTS; i++)
    {
        block_t *list = (block_t *)&free_lists[i];

        // Loop through each free block in the list
        for (block_t *blk = free_block_get_next(list); blk != list; blk = free_block_get_next(blk))
        {
            // Find the first fit
            if (block_size(blk) >= size)
            {
                // Remove from the list
                free_blocks_remove(blk);

                return blk;
            }
        }
    }

    return NULL;
}

/**
 * @brief Figure 17. malloc - Allocate a memory for the given required payload size.
 * This searches for the smallest free block from the segregated free list and sets
 * the appropriate flags. 
 * 
 * @param size_t payload size
 * 
 * @return new pointer to the allocated block
 */
void *malloc(size_t size)
{
    // Calculate the block size based on the payload size
    size_t blk_size = align(sizeof(header_t) + size);

    // Figure 18. Find the block in the segregated free lists
    block_t *blk = block_find(blk_size);
    if (!blk)
    {
        // There is no free block that will fit the required size
        // Resize the heap to accomodate the new memory size 
        size_t new_blk_size = align_page(blk_size);
        block_t *end = mm_sbrk(align_page(new_blk_size));
        if (!end)
        {
            return NULL;
        }
        block_t *end_block = (block_t *)((char *)end - sizeof(header_t));
        blk = end_block;

        // Set new free block properties
        block_set_props(blk, new_blk_size, block_prev_allocated(end_block), false, NULL, NULL);

        // Update the end block
        end_block = (block_t *)((char *)end_block + new_blk_size);
        block_set_props(end_block, END_BLOCK_SIZE, false, true, NULL, NULL);
    }

    dbg_assert(block_size(blk) >= blk_size);

    // Figure 19. Set the block as allocated
    size_t rem_size = block_size(blk) - blk_size;
    block_set_props(blk, blk_size, block_prev_allocated(blk), true, NULL, NULL);

    // Split the free block if there is a remaining size
    if (rem_size > 0)
    {
        // Updated the remaining block
        block_t *rem_blk = (block_t *)((char *)blk + blk_size);
        block_set_props(rem_blk, rem_size, true, false, NULL, NULL);

        // Insert to the free list
        free_blocks_insert(rem_blk);

        // Set the next block
        block_t *next_blk = (block_t *)((char *)rem_blk + rem_size);
        block_set_props(next_blk, block_size(next_blk), false, block_allocated(next_blk),
                        block_allocated(next_blk) ? NULL : free_block_get_prev(next_blk),
                        block_allocated(next_blk) ? NULL : free_block_get_next(next_blk));
    }
    else
    {
        // Set the next block header prev allocated flag
        block_t *next_blk = (block_t *)((char *)blk + blk_size);
        block_set_props(next_blk, block_size(next_blk), true, block_allocated(next_blk),
                        block_allocated(next_blk) ? NULL : free_block_get_prev(next_blk),
                        block_allocated(next_blk) ? NULL : free_block_get_next(next_blk));
    }

    // Figure 20. Check heap
    dbg_assert(mm_checkheap(__LINE__));

    return &blk->allocated_blk.payload;
}

/**
 * @brief Figure 21. free a block using the payload pointer. After the allocated block
 * is freed, it is coalesced with adjacent free blocks.
 * 
 * @param void* payload pointer to free
 */
void free(void *ptr)
{
    // Do nothing if pointer is null
    if (!ptr)
    {
        return;
    }

    // Set the block as free
    block_t *blk = (block_t *)((char *)ptr - sizeof(header_t));
    size_t size = block_size(blk);

    // Update the flags of this block
    block_set_props(blk, block_size(blk), block_prev_allocated(blk), false, NULL, NULL);

    // Figure 22. Coalesce with previous block if free
    if (!block_prev_allocated(blk))
    {
        size_t prev_blk_size = block_size((block_t *)((char *)blk - sizeof(header_t)));
        block_t *prev_blk = (block_t *)((char *)blk - prev_blk_size);

        // Remove from list
        free_blocks_remove(prev_blk);

        // Update the flags of the previous block
        block_set_props(prev_blk, size + prev_blk_size, block_prev_allocated(prev_blk), false, NULL, NULL);

        // Set the blk
        blk = prev_blk;
    }

    // Figure 23. Coalesce with next block if free
    block_t *next_blk_free = (block_t *)((char *)blk + block_size(blk));
    if (!block_allocated(next_blk_free))
    {
        // Remove from list
        free_blocks_remove(next_blk_free);

        // Update the flags of the block
        block_set_props(blk, block_size(blk) + block_size(next_blk_free), block_prev_allocated(blk), false, NULL, NULL);
    }

    // Figure 24. Update the next block prev allocated flag
    block_t *next_blk = (block_t *)((char *)blk + block_size(blk));
    block_set_props(next_blk, block_size(next_blk), false, block_allocated(next_blk),
                    block_allocated(next_blk) ? NULL : free_block_get_prev(next_blk),
                    block_allocated(next_blk) ? NULL : free_block_get_next(next_blk));

    // Insert to the free list
    free_blocks_insert(blk);

    // Check heap
    dbg_assert(mm_checkheap(__LINE__));
}

/**
 * @brief Figure 25. reallocate the block based on the new size. If the new size is
 * less than the block size, the same payload pointer is returned. If the new size is
 * more than the block size, the malloc function is called to get a new memory location
 * and the payload is copied to the new memory region.
 * 
 * @param void* old pointer
 * @param size_t new payload size
 * 
 * @return new pointer to the reallocated block
 */
void *realloc(void *oldptr, size_t size)
{
    if (size == 0)
    {
        // Free if block is size is zero
        free(oldptr);
        return NULL;
    }
    else
    {
        // Figure 26. Get the block
        block_t *blk = (block_t *)((char *)oldptr - sizeof(header_t));
        size_t old_size = block_size(blk) - sizeof(header_t);
        size_t new_block_size = align(size + sizeof(header_t));

        if (new_block_size <= old_size)
        {
            // Splitting the reduced block size does not improve the space utilization
            // So, I'm keeping this part simple
            // Use the same block
            return oldptr;
        }
        else
        {
            // Figure 27. Find a new memory location
            void *newptr = malloc(size);
            memcpy(newptr, oldptr, old_size);

            // Free the previous block
            free(oldptr);

            return newptr;
        }
    }
}

/*
 * calloc
 * This function is not tested by mdriver, and has been implemented for you.
 */
void *calloc(size_t nmemb, size_t size)
{
    void *ptr;
    size *= nmemb;
    ptr = malloc(size);
    if (ptr)
    {
        memset(ptr, 0, size);
    }
    return ptr;
}

/*
 * Returns whether the pointer is in the heap.
 * May be useful for debugging.
 */
static bool in_heap(const void *p)
{
    return p <= mm_heap_hi() && p >= mm_heap_lo();
}

/*
 * Returns whether the pointer is aligned.
 * May be useful for debugging.
 */
static bool aligned(const void *p)
{
    size_t ip = (size_t)p;
    return align(ip) == ip;
}

/*
 * mm_checkheap
 * You call the function via mm_checkheap(__LINE__)
 * The line number can be used to print the line number of the calling
 * function where there was an invalid heap.
 */
bool mm_checkheap(int line_number)
{
#ifdef DEBUG
    // Figure 28. Check the start block
    block_t *start_block = mm_heap_lo();

    // Check if the start block size is correct
    if (block_size(start_block) != sizeof(header_t))
    {
        dbg_printf("Start block size should be %ld, got %ld.\n", sizeof(header_t), block_size(start_block));
        return false;
    }
    
    // Check if the start block PAF flag is correct
    if (!block_prev_allocated(start_block))
    {
        dbg_printf("Start block prev allocated flag should be true.\n");
        return false;
    }

    // Check if the start block AF flag is correct
    if (!block_allocated(start_block))
    {
        dbg_printf("Start block allocated flag should be true.\n");
        return false;
    }

    // Figure 29. Check the end block
    block_t *end_block = (block_t *)((char *)mm_heap_hi() + 1 - sizeof(header_t));

    // Check if the end block size is correct
    if (block_size(end_block) != 0)
    {
        dbg_printf("End block size should be %ld, got %ld.\n", (size_t)0, block_size(end_block));
        return false;
    }

    // Check if the end block AF flag is correct
    if (!block_allocated(end_block))
    {
        dbg_printf("End block allocated flag should be true.\n");
        return false;
    }

    // Figure 30. Check adjacent blocks
    bool prev_allocated = true;
    int num_free_blocks = 0;
    block_t *blk = (block_t *)((char *)start_block + sizeof(header_t));

    // Loop through each adjacent block in the heap
    for (; blk < end_block; blk = (block_t *)((char *)blk + block_size(blk)))
    {
        // Check the size if aligned
        size_t size = block_size(blk);
        if (!aligned((const void *)size))
        {
            dbg_printf("Block size of %p is not aligned.\n", blk);
            return false;
        }

        // Check the prev allocated flag
        if (block_prev_allocated(blk) != prev_allocated)
        {
            dbg_printf("Prev allocated of %p != %d.\n", blk, prev_allocated);
            return false;
        }

        // Update the prev allocated flag
        prev_allocated = block_allocated(blk);

        // Figure 31. For free block, check if the header and footer flags matches
        if (!prev_allocated)
        {
            header_t header = *((header_t *)blk);
            header_t footer = *((header_t *)((char *)blk + size - sizeof(header_t)));

            // Header and footer should exactly match for size > SMALL_FREE_BLOCK_SIZE
            if (size > SMALL_FREE_BLOCK_SIZE)
            {
                if (header != footer)
                {
                    dbg_printf("Header and footer of %p do not match. Header = %lx, footer = %lx.\n", blk, header, footer);
                    return false;
                }
            }
            else
            {
                // Get the header and footer flags of the small free block
                header_t header_flag = header & FLAG_MASK;
                header_t footer_flag = footer & FLAG_MASK;

                // Only check for flags
                if (header_flag != footer_flag)
                {
                    dbg_printf("Header and footer flags of %p do not match. Header flag = %lx, footer flag = %lx.\n", blk, header_flag, footer_flag);
                    return false;
                }
            }

            // Update the count
            num_free_blocks++;
        }
    }

    // Figure 32. Check the free lists
    int counted_free_blocks = 0;
    for (int i = 0; i < NUM_FREE_LISTS; i++)
    {
        for (block_t *blk = free_block_get_next((block_t *)&free_lists[i]); blk != (block_t *)&free_lists[i]; blk = free_block_get_next(blk))
        {
            // Check free block if null
            if (block_allocated(blk))
            {
                dbg_printf("Allocated block %p in free list.\n", blk);
                return false;
            }

            // Update the count
            counted_free_blocks++;
        }
    }

    // Match the number of free blocks
    if (num_free_blocks != counted_free_blocks)
    {
        dbg_printf("Some free blocks not in the free lists.\n");
        return false;
    }

#endif // DEBUG
    return true;
}

