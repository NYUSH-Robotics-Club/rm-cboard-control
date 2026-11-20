#ifndef VTM_DMA_CURSOR_H
#define VTM_DMA_CURSOR_H
#include <stdint.h>
#define VTM_DMA_SIZE 1024U
/* NDTR can transiently be zero at reload. With a completed/pending wrap
 * already counted, zero means position zero, not another complete buffer. */
static inline uint32_t VtmDma_Producer(uint32_t wraps, uint32_t remaining)
{
    return wraps * VTM_DMA_SIZE + ((VTM_DMA_SIZE - remaining) & (VTM_DMA_SIZE - 1U));
}
#endif
