// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include "mempool.h"


struct mempool * mempool_create(uint32_t block_size)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	struct mempool * new = (struct mempool *)MALLOC(MEMPOOL_ALIGNED(sizeof(struct mempool)));

	if (!new)
		return NULL;

	if (!IS_MEMPOOL_ALIGNED((long)new)) {

		printf("Nonaligned\n");
		free(new);
		new = (struct mempool *)MALLOC(MEMPOOL_ALIGNED(sizeof(struct mempool)));
	}

	if (!new)
		return NULL;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0) 
	portMUX_INITIALIZE(&(new->mutex));
#else
	vPortCPUInitializeMutex(&(new->mutex));
#endif
	new->block_size = MEMPOOL_ALIGNED(block_size);
	SLIST_INIT(&(new->head));

	return new;
#else
	return NULL;
#endif
}

void mempool_destroy(struct mempool* mp)
{
#ifdef CONFIG_ESP_CACHE_MALLOC
	void * node1 = NULL;

	if (!mp)
		return;

	while ((node1 = SLIST_FIRST(&(mp->head))) != NULL) {
		SLIST_REMOVE_HEAD(&(mp->head), entries);
		FREE(node1);
	}
	SLIST_INIT(&(mp->head));

	FREE(mp);
#endif
}

void * mempool_alloc(struct mempool* mp, int nbytes, int need_memset)
{
	void *buf = NULL;

#ifdef CONFIG_ESP_CACHE_MALLOC
	if (!mp || mp->block_size < nbytes)
		return NULL;

	portENTER_CRITICAL(&(mp->mutex));
	if (!SLIST_EMPTY(&(mp->head))) {
		buf = SLIST_FIRST(&(mp->head));
		SLIST_REMOVE_HEAD(&(mp->head), entries);
		portEXIT_CRITICAL(&(mp->mutex));
	} else {
		portEXIT_CRITICAL(&(mp->mutex));
		buf = MEM_ALLOC(mp->block_size);
	}
#else
	buf = MEM_ALLOC(MEMPOOL_ALIGNED(nbytes));
#endif

	if (buf && need_memset)
		memset(buf, 0, nbytes);

	return buf;

}

void mempool_free(struct mempool* mp, void *mem)
{
	if (!mem)
		return;
#ifdef CONFIG_ESP_CACHE_MALLOC
	if (!mp)
		return;

	portENTER_CRITICAL(&(mp->mutex));
	SLIST_INSERT_HEAD(&(mp->head), (struct mempool_entry *)mem, entries);
	portEXIT_CRITICAL(&(mp->mutex));

#else
	FREE(mem);
#endif
}

