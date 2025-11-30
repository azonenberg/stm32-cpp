/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2020-2025 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef PageTable_h
#define PageTable_h

#ifdef __aarch64__

/**
	@brief An aarch64 page table
 */
template<uint64_t entrySize, uint32_t numEntries = 512>
class PageTable
{
public:

	///@brief Initialize the page table to all zeroes (invalid/unmapped)
	PageTable()
	{ }

	void Clear()
	{
		//Can't use memset since we probably haven't configured the MMU yet
		for(uint32_t i=0; i<numEntries; i++)
			m_entries[i] = 0;
	}

	///@brief Fill out an entry for a leaf page
	void SetLeafEntry(uint64_t vaddr, uint64_t paddr, bool nx, mairidx_t attribs)
	{
		uint32_t idx = vaddr / entrySize;

		//Fill out the base descriptor
		uint64_t descriptor =
			paddr |				//Block address
			(1 << 10) |			//AF pre set so we don't fault
			(attribs << 2) |	//Attributes
			1;					//Valid bit

		//Add NX bit if requested
		if(nx)
			descriptor |= (1LL << 53);

		//bit 9:8: SH (shareability), default to zero (non-shareable)
		//for normal memory, mark as inner shareable
		if(attribs == MAIR_IDX_NORMAL)
			descriptor |= (3 << 8);

		m_entries[idx] = descriptor;
	}

	///@brief Fill out an entry for a child page
	void SetChildEntry(uint64_t vaddr, PageTable<entrySize/512>& child)
	{
		uint32_t idx = vaddr / entrySize;
		uint64_t paddr = reinterpret_cast<uint64_t>(child.GetData());

		//Fill out the base descriptor
		uint64_t descriptor =
			paddr |				//Table address
			(1 << 10) |			//AF pre set so we don't fault
			2 |					//This is a descriptor not a leaf entry
			1;					//Valid bit

		m_entries[idx] = descriptor;
	}

	///@brief Get a pointer to the page table itself
	const uint64_t* GetData()
	{ return m_entries; }

	///@brief Get the number of bytes addressed by one page table entry
	uint64_t GetEntrySize()
	{ return entrySize; }

protected:

	///@brief The actual entries in the page table
	uint64_t m_entries[numEntries];
};

#endif
#endif
