#ifndef __LIB_RINGBUFFER_H__
#define __LIB_RINGBUFFER_H__

struct RingBuffer{
	unsigned char*	m_base;
	unsigned char*	m_writePos;
	unsigned char*	m_readPos;
	unsigned char*	m_data;
	int				m_size;
	int 			m_full;
};
int             RingBuffer_Init(struct RingBuffer* ring, int size, void* mem_region);
int             RingBuffer_Destroy(struct RingBuffer* ring);
void 			RingBuffer_Clear(struct RingBuffer* ring);
unsigned char 	RingBuffer_GetAt(struct RingBuffer* ring, int index);
//int				DataLength();
//int				SpaceLength();
int				RingBuffer_Pop(struct RingBuffer* ring, void* data, int maxlen);
int				RingBuffer_Push(struct RingBuffer* ring, const void* data, int datalen);
#endif
