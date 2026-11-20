#include "vtm_dma_cursor.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    assert(VtmDma_Producer(0U, 1024U) == 0U);
    assert(VtmDma_Producer(0U, 1021U) == 3U);
    assert(VtmDma_Producer(1U, 0U) == 1024U);
    assert(VtmDma_Producer(1U, 1024U) == 1024U);
    assert(VtmDma_Producer(1U, 1023U) == 1025U);
    assert(VtmDma_Producer(2U, 948U) - 1024U == 1100U);
    assert(VtmDma_Producer(4194304U, 1024U) == 0U);
    assert((uint32_t)(VtmDma_Producer(4194304U, 1023U) - UINT32_MAX) == 2U);
    puts("VTM DMA cursor: PASS (empty, reload-zero, pending wrap, overwrite distance, counter wrap)");
    return 0;
}
