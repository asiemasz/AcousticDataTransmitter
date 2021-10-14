#ifndef DMA_H
#define DMA_H

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

enum DMA_STREAM {
    DMA_STREAM_0,
    DMA_STREAM_1,
    DMA_STREAM_2,
    DMA_STREAM_3,
    DMA_STREAM_4,
    DMA_STREAM_5,
    DMA_STREAM_6,
    DMA_STREAM_7
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

enum DMA_DIRECT_MODE {
    DMA_DIRECT_MODE_ENABLED,
    DMA_DIRECT_MODE_DISABLED
};

enum DMA_FIFO_THRESHOLD {
    DMA_FIFO_THRESHOLD_1_4_FULL,
    DMA_FIFO_THRESHOLD_1_2_FULL,
    DMA_FIFO_THRESHOLD_3_4_FULL,
    DMA_FIFO_THRESHOLD_FULL
};


#endif
