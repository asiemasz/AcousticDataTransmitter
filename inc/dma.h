#ifndef DMA_H
#define DMA_H

#include "stm32f401xe.h"
#include <assert.h>

enum DMA_CHANNEL {
    DMA_CHANNEL_0,
    DMA_CHANNEL_1,
    DMA_CHANNEL_2,
    DMA_CHANNEL_3,
    DMA_CHANNEL_4,
    DMA_CHANNEL_5,
    DMA_CHANNEL_6,
    DMA_CHANNEL_7
};

enum DMA_INCREMENTAL_BURST {
    DMA_INCREMENTAL_BURST_DISABLED,
    DMA_INCREMENTAL_BURST_4_BEATS,
    DMA_INCREMENTAL_BURST_8_BEATS,
    DMA_INCREMENTAL_BURST_16_BEATS
};

enum DMA_PRIORITY_LEVEL {
    DMA_PRIORITY_LEVEL_LOW,
    DMA_PRIORITY_LEVEL_MEDIUM,
    DMA_PRIORITY_LEVEL_HIGH,
    DMA_PRIORITY_LEVEL_VERY_HIGH
};

enum DMA_DATA_SIZE {
    DMA_DATA_SIZE_BYTE,
    DMA_DATA_SIZE_HALF_WORD,
    DMA_DATA_SIZE_WORD
};

enum DMA_DIRECTION {
    DMA_DIRECTION_PERIPHERAL_TO_MEMORY,
    DMA_DIRECTION_MEMORY_TO_PERIPHERAL,
    DMA_DIRECTION_MEMORY_TO_MEMORY
};

enum DMA_MODE {
    DMA_DIRECT_MODE,
    DMA_CIRCULAR_MODE
};

enum DMA_FIFO_MODE {
		DMA_FIFO_MODE_DISABLED,
		DMA_FIFO_MODE_ENABLED
};

enum DMA_FIFO_THRESHOLD {
    DMA_FIFO_THRESHOLD_1_4_FULL,
    DMA_FIFO_THRESHOLD_1_2_FULL,
    DMA_FIFO_THRESHOLD_3_4_FULL,
    DMA_FIFO_THRESHOLD_FULL
};

enum DMA_INC {
    DMA_INC_DISABLE,
    DMA_INC_ENABLE
};

typedef struct DMA_requestStruct {
	enum DMA_CHANNEL 		channel;
	enum DMA_DIRECTION	direction;
	enum DMA_MODE 			mode;
	enum DMA_DATA_SIZE	periphDataSize;
	enum DMA_DATA_SIZE	memoryDataSize;
	enum DMA_PRIORITY_LEVEL priority;
    enum DMA_INC memInc;
    enum DMA_INC periphInc;
	enum DMA_FIFO_MODE	fifoMode;
	enum DMA_FIFO_THRESHOLD fifoThreshold;
    volatile uint32_t* peripheralAddress;
    volatile uint32_t* memory0Address;
    volatile uint32_t* memory1Address;
    uint16_t dataNumber;
} DMA_requestStruct;

enum DMA_IT {
	DMA_IT_DIRECT_ERROR = 2,
	DMA_IT_TRANSFER_ERROR = 4,
	DMA_IT_HALF_TRANSFER = 8,
	DMA_IT_TRANSFER_COMPLETE = 16
};

enum DMA_IT_FLAG {
	DMA_IT_FLAG_FIFO_ERROR = 0,
	DMA_IT_FLAG_DIRECT_MODE_ERROR = 2,
	DMA_IT_FLAG_TRANSFER_ERROR = 3,
	DMA_IT_FLAG_HALF_TRANSFER = 4,
	DMA_IT_FLAG_TRANSFER_COMPLETE = 5
};

void dma_init(DMA_TypeDef* dma);
    
void dma_streamConfig(DMA_Stream_TypeDef* stream, DMA_requestStruct* request);

void dma_streamITEnable(DMA_Stream_TypeDef* stream, uint8_t its);

void dma_streamITDisable(DMA_Stream_TypeDef* stream, uint8_t its);

void dma_streamClearITFlag(DMA_TypeDef* dma, uint8_t stream, enum DMA_IT_FLAG flag);

uint8_t dma_streamGetITFlag(DMA_TypeDef* dma, uint8_t stream, enum DMA_IT_FLAG flag);

#endif
