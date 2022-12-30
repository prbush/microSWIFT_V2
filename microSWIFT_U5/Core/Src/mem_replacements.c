/*
 * mem_replacements.c
 *
 *
 */

#include "mem_replacements.h"


static TX_BYTE_POOL memory_pool = {0};

int memory_pool_init(VOID* pool_start, size_t pool_size)
{
	UINT ret = tx_byte_pool_create(&memory_pool, "waves mem pool", pool_start, pool_size);

	return (ret == TX_SUCCESS) ? 0 : -1;
}

void* malloc_replacement(size_t size)
{
	CHAR *pointer = TX_NULL;
	if (size > 0) {
		UINT ret = tx_byte_allocate(&memory_pool, (VOID**) &pointer, (ULONG)size, TX_NO_WAIT);
		if (ret != TX_SUCCESS){
		  return NULL;
		}
	}
	return (void*)pointer;
}


void* calloc_replacement(size_t num, size_t size)
{
	CHAR *pointer = TX_NULL;
	if (size > 0) {
		UINT ret = tx_byte_allocate(&memory_pool, (VOID**) &pointer, (ULONG)(num * size), TX_NO_WAIT);
		if (ret != TX_SUCCESS){
		  return NULL;
		}

		memset(pointer, 0, size);
	}
	return (void*)pointer;
}


void free_replacement(VOID* ptr)
{
	tx_byte_release(ptr);
}


