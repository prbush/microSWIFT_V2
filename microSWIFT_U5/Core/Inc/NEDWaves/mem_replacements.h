/*
 * mem_replacements.h
 *
 *  Created on: Dec 29, 2022
 *      Author: veteran
 */

#ifndef INC_WAVES_MEM_REPLACEMENTS_H_
#define INC_WAVES_MEM_REPLACEMENTS_H_

#define NUM_SAMPLES 8192

#include "stddef.h"
#include "math.h"
#include "tx_api.h"
#include "NEDwaves_memlight_types.h"
#include "NEDwaves_memlight_emxAPI.h"
#include "configuration.h"

void waves_memory_pool_init(TX_BYTE_POOL* pool);
UINT waves_memory_pool_create(VOID* pool_start, size_t pool_size);
UINT waves_memory_pool_delete();
float* get_waves_float_array(microSWIFT_configuration* config);
void* malloc_replacement(size_t size);
void* calloc_replacement(size_t num, size_t size);
void free_replacement(void* ptr);

// NEDWaves support
emxArray_real32_T *argInit_1xUnbounded_real32_T(microSWIFT_configuration* config);
double argInit_real_T(void);


#endif /* INC_WAVES_MEM_REPLACEMENTS_H_ */
