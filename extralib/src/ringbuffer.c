#include <ringbuffer.h>

inline int				RingBuffer_DataLength(struct RingBuffer* ring){
	int ret = 0;
	if(ring->m_data){
		if(ring->m_readPos == ring->m_writePos){
			if(ring->m_full)
				ret = ring->m_size;
			else
				ret = 0;
		}else{
			if(ring->m_readPos > ring->m_writePos){
				ret = ring->m_size + ring->m_writePos - ring->m_readPos;
			}else{
				ret = ring->m_writePos - ring->m_readPos;
			}
		}
	}
	return ret;
}
inline int				RingBuffer_SpaceLength(struct RingBuffer* ring){
	int ret = 0;
	if(ring->m_data){
		if(ring->m_readPos == ring->m_writePos){
			if(ring->m_full)
				ret = 0;
			else
				ret = ring->m_size;
		}else{
			if(ring->m_writePos > ring->m_readPos){
				ret = ring->m_size + ring->m_readPos - ring->m_writePos;
			}else{
				ret = ring->m_readPos - ring->m_writePos;
			}
		}
	}
	return ret;
}

int RingBuffer_Init(struct RingBuffer* ring, int size, void* mem_region){
	ring->m_base		= 0;
	ring->m_writePos	= 0;
	ring->m_readPos	    = 0;
	ring->m_data		= 0;
	ring->m_full		= 0;
	ring->m_size		= 0;

	if(size > 0){
		ring->m_size = size;
		ring->m_base = (unsigned char*)mem_region;
		if(ring->m_base){
			ring->m_data 		= ring->m_base;
			ring->m_writePos 	= ring->m_base;
			ring->m_readPos	    = ring->m_base;
		}
	}
    return 0;
}
int RingBuffer_Destroy(struct RingBuffer* ring){
	if(ring->m_base){
		ring->m_base 		= 0;
		ring->m_writePos	= 0;
		ring->m_readPos	= 0;
		ring->m_data		= 0;
		ring->m_full		= 0;
		ring->m_size		= 0;
	}
	return 0;
}

void 			RingBuffer_Clear(struct RingBuffer* ring){
	ring->m_data 		= ring->m_base;
	ring->m_writePos 	= ring->m_base;
	ring->m_readPos	    = ring->m_base;
	ring->m_full		= 0;
}
unsigned char 	RingBuffer_GetAt(struct RingBuffer* ring, int index){
	int 			len = DataLength(ring);
	unsigned char*	p	= ring->m_readPos;
	unsigned char	ret = 0;
	if(ring->m_data){
		if(index >= 0 && index < len){
			while(index > 0){
				p++;
				if(p > ring->m_data + ring->m_size - 1)
					p = ring->m_data;
				index--;
			}
		}
		ret = *p;
	}
	return ret;
}
int				RingBuffer_Pop(struct RingBuffer* ring, void* data, int maxlen){
	int ret = 0;
	int size = 0;
	int index;
	unsigned char*	p = (unsigned char*)data;

	if(ring->m_data && maxlen > 0)
	{
		size = RingBuffer_DataLength(ring);
		size = (size > maxlen) ? maxlen : size;

		for(index = 0; index < size; index++)
		{
			if(p)
				p[index] = *ring->m_readPos;
			ring->m_readPos++;
			if(ring->m_readPos > ring->m_data + ring->m_size - 1)
				ring->m_readPos = ring->m_data;
		}
		if(ring->m_readPos  == ring->m_writePos)
			ring->m_full = 0;
		ret = size;
	}
	return ret;
}
int				RingBuffer_Push(struct RingBuffer* ring, const void* data, int datalen){
	int ret = 0;
	int index;
	int size;
	unsigned char*	p = (unsigned char*)data;

	if(ring->m_data && data && datalen > 0)
	{
		size = RingBuffer_SpaceLength(ring);
		if(size > 0)
		{
			size = (size > datalen) ? datalen : size;
			for(index = 0; index < size; index ++ )
			{
				*ring->m_writePos = p[index];
				ring->m_writePos ++;
				if(ring->m_writePos > ring->m_data + ring->m_size-1)
					ring->m_writePos = ring->m_data;
			}
			if(ring->m_writePos == ring->m_readPos)
				ring->m_full = 1;
			ret = size;
		}
	}
	return ret;
}
