/*
 * mem_replacements.c
 *
 *
 */

#include "NEDWaves/mem_replacements.h"


TX_BYTE_POOL* memory_pool;

void waves_memory_pool_init(TX_BYTE_POOL* pool)
{
	memory_pool = pool;
}

UINT waves_memory_pool_create(VOID* pool_start, size_t pool_size)
{
	UINT ret = tx_byte_pool_create(memory_pool, "waves mem pool", pool_start, pool_size);

	return ret;
}

UINT waves_memory_pool_delete()
{
	UINT ret = tx_byte_pool_delete(memory_pool);

	return ret;
}

float* get_waves_float_array(microSWIFT_configuration* config)
{
	return (float*) calloc_replacement(config->samples_per_window, sizeof(float));
}

void* malloc_replacement(size_t size)
{
	CHAR *pointer = TX_NULL;
	if (size > 0) {
		UINT ret = tx_byte_allocate(memory_pool, (VOID**) &pointer, (ULONG)size, TX_NO_WAIT);
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
		UINT ret = tx_byte_allocate(memory_pool, (VOID**) &pointer, (ULONG)(num * size), TX_NO_WAIT);
		if (ret != TX_SUCCESS){
		  return NULL;
		}

		memset(pointer, 0, (num * size));
	}
	return (void*)pointer;
}


void free_replacement(VOID* ptr)
{
	tx_byte_release(ptr);
}


emxArray_real32_T *argInit_1xUnbounded_real32_T(microSWIFT_configuration* config)
{
	emxArray_real32_T *result;
	result = emxCreate_real32_T(1, config->samples_per_window);
	return result;
}

double argInit_real_T(void)
{
  return 0.0;
}

