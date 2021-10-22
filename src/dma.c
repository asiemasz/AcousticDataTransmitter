#include "../inc/dma.h"

void dma_init(DMA_TypeDef* dma) {
	if(dma == DMA1) {
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	}
	else if(dma == DMA2) {
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	}	
	//DMA2_Stream4->CR |= DMA_SxCR_HTIE;
}

void dma_streamConfig(DMA_Stream_TypeDef* stream, DMA_requestStruct* request) {
	//if stream is activated disable it
	if(stream->CR & DMA_SxCR_EN) {
		stream->CR &= ~DMA_SxCR_EN;
	}//wait till stream is disabled
	while(stream->CR & DMA_SxCR_EN);

	//select channel to be configured
	stream->CR |= (uint32_t) request->channel << DMA_SxCR_CHSEL_Pos;

	//select priority level
	stream->CR |= (uint32_t) request->priority << DMA_SxCR_PL_Pos;

	//set memory and peripheral data size
	stream->CR |= (uint32_t) request->memoryDataSize << DMA_SxCR_MSIZE_Pos;
	stream->CR |= (uint32_t) request->periphDataSize << DMA_SxCR_PSIZE_Pos;

	//select data transfer direction
	stream->CR |= (uint32_t) request->direction << DMA_SxCR_DIR_Pos;

	//set number of data to be transferred
	stream->NDTR |= request->dataNumber & DMA_SxNDT_Msk;

	//set addresses 
	switch(request->direction) {
		case DMA_DIRECTION_PERIPHERAL_TO_MEMORY:
		case DMA_DIRECTION_MEMORY_TO_PERIPHERAL:
			stream->PAR |= (uint32_t)request->peripheralAddress;
			stream->M0AR |= (uint32_t)request->memory0Address;
			break;
		case DMA_DIRECTION_MEMORY_TO_MEMORY:
			stream->M0AR |= (uint32_t)request->memory0Address;
			stream->M1AR |= (uint32_t)request->memory1Address;
			break;
	}
	if(request->memInc) {
		stream->CR |= DMA_SxCR_MINC;
	}
	if(request->periphInc) {
		stream->CR |= DMA_SxCR_PINC;
	}
	
	if(request->mode == DMA_CIRCULAR_MODE) {
		stream->CR |= DMA_SxCR_CIRC;
		//stream->FCR |= DMA_SxFCR_DMDIS;
		stream->FCR |= (uint32_t) request->fifoThreshold << DMA_SxFCR_FTH_Pos;
	}
	else {
		stream->CR &= ~DMA_SxCR_CIRC;
		stream->FCR &= ~DMA_SxFCR_DMDIS;
	}

	//enable stream
	stream->CR |= DMA_SxCR_EN;
}

void dma_streamITEnable(DMA_Stream_TypeDef* stream, uint8_t its) {
	stream->CR |= its;
}

void dma_streamClearITFlag(DMA_TypeDef* dma, uint8_t stream, enum DMA_IT_FLAG flag) {
	assert(stream >= 0 && stream < 8);
	if(stream < 4) {
		dma->LIFCR |= (1U << (flag + (stream * 6) + 4*stream/2));
	}
	else {
		stream = stream - 4;
		dma->HIFCR |= (1U << (flag + (stream * 6) + 4*stream/2));
	}
}

uint8_t dma_streamGetITFlag(DMA_TypeDef* dma, uint8_t stream, enum DMA_IT_FLAG flag) {
	assert(stream >= 0 && stream < 8);
	if(stream < 4) {
		return ((dma->LISR & (1U << (flag + (stream * 6) + 4*stream/2))) > 0);
	}
	else {
		stream = stream - 4;
		return ((dma->HISR & (1U << (flag + (stream * 6) + 4*stream/2))) > 0);
	}
}

