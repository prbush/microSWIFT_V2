/*
 * mem_replacements.h
 *
 *  Created on: Dec 29, 2022
 *      Author: veteran
 */

#ifndef INC_WAVES_MEM_REPLACEMENTS_H_
#define INC_WAVES_MEM_REPLACEMENTS_H_

#include "stddef.h"
#include "tx_api.h"

int memory_pool_init(VOID* pool_start,UINT size);
void* malloc_replacement(size_t size);
void* calloc_replacement(size_t num, size_t size);
void free_replacement(void* ptr);

#endif /* INC_WAVES_MEM_REPLACEMENTS_H_ */
