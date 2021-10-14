#include "../inc/dma.h"

void dma_init(DMA_TypeDef* dma) {
	if(dma == DMA1) {
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	}
	else if(dma == DMA2) {
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	}
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
			stream->PAR |= request->peripheralAddress;
			stream->M0AR |= request->memory0Address;
			break;
		case DMA_DIRECTION_MEMORY_TO_MEMORY:
			stream->M0AR |= request->memory0Address;
			stream->M1AR |= request->memory1Address;
			break;
	}
	
	if(request->mode == DMA_CIRCULAR_MODE) {
		stream->CR |= DMA_SxCR_CIRC;
		stream->FCR |= DMA_SxFCR_DMDIS;
		stream->FCR |= (uint32_t) request->fifoThreshold << DMA_SxFCR_FTH_Pos;
	}
	else {
		stream->CR &= ~DMA_SxCR_CIRC;
		stream->FCR &= ~DMA_SxFCR_DMDIS;
	}

	//enable stream
	stream->CR |= DMA_SxCR_EN;
}
