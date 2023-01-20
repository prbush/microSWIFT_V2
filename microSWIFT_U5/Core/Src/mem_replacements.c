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

		memset(pointer, 0, (num * size));
	}
	return (void*)pointer;
}


void free_replacement(VOID* ptr)
{
	tx_byte_release(ptr);
}


emxArray_real32_T *argInit_1xUnbounded_real32_T_down(void)
{
	emxArray_real32_T *result;
	float *result_data;
	int idx0;
	int idx1;
	/* Set the size of the array.
	Change this size to the value that the application requires. */
	result = emxCreate_real32_T(1, NUM_SAMPLES);
	result_data = result->data;
	/* Loop over the array to initialize each element. */
	for (idx0 = 0; idx0 < 1; idx0++) {
		for (idx1 = 0; idx1 < result->size[0U]; idx1++) {
		/* Set the value of the array element.
		Change this value to the value that the application requires. */
		result_data[idx1] = -2.51327 * sin(-0.12566*idx1);
		}
	}
	return result;
}


emxArray_real32_T *argInit_1xUnbounded_real32_T_north_east(void)
{
	emxArray_real32_T *result;
	float *result_data;
	int idx0;
	int idx1;
	/* Set the size of the array.
	Change this size to the value that the application requires. */
	result = emxCreate_real32_T(1, NUM_SAMPLES);
	result_data = result->data;
	/* Loop over the array to initialize each element. */
	for (idx0 = 0; idx0 < 1; idx0++) {
		for (idx1 = 0; idx1 < result->size[0U]; idx1++) {
		/* Set the value of the array element.
		Change this value to the value that the application requires. */
		result_data[idx1] = 0.707 * 2.51327 * cos(-0.12566*idx1);
		}
	}
	return result;
}


emxArray_real32_T *argInit_1xUnbounded_real32_T(float* data)
{
	emxArray_real32_T *result;
	int idx0;
	int idx1;
	/* Set the size of the array.
	Change this size to the value that the application requires. */
	result = emxCreate_real32_T(1, NUM_SAMPLES);
	result->data = data;
	return result;
}

double argInit_real_T(void)
{
  return 0.0;
}

