 // Copyright (c) 2019 by FEI Company
#pragma once

#include <fstream>
#include <vector>

// FeiBitStremer.h
// Copyright (c) 2019 by FEI Company
#include <queue>
#include <iostream>
#include "stdint.h"


typedef uint8_t BitType;
typedef uint16_t MultiBitType;

typedef uint64_t BitStreamWordType;
const int BitStreamWordTypeNBits = 8 * sizeof(BitStreamWordType);

// NB: nWords not used, not checked anyware; responsibility of caller now.
class BitStreamer
{
public:
	BitStreamer(BitStreamWordType* buffer) :
		buffer(buffer), ptr(buffer), bitCounter(BitStreamWordTypeNBits)
	{
		curValue = *ptr;
	}

	// todo this can be optimized.
	MultiBitType getBits(unsigned NBits)
	{
		MultiBitType r = curValue & ((1<<NBits)-1);
		if (bitCounter <= NBits)
		{
			// take a new one.
			curValue = *(++ptr);
			auto xtraBitsToRead = NBits - bitCounter; //# Remaining bits.
			r |= ((curValue & ((1<<xtraBitsToRead)-1)) << (bitCounter));
			curValue >>= xtraBitsToRead;
			bitCounter = BitStreamWordTypeNBits - xtraBitsToRead;
		}
		else
		{
			// just shift and done!
			curValue >>= NBits;
			bitCounter -= NBits;
		}
		return r;
	}
 
   void rewind(unsigned NBits)
   {
       //std::cout<<"REWIND "<<bitCounter<<", "<<curValue<<std::endl;
       bitCounter += NBits;
       if (bitCounter > BitStreamWordTypeNBits)
       {
           bitCounter -= BitStreamWordTypeNBits;
           --ptr;
       }
       curValue = (*ptr) >> (BitStreamWordTypeNBits-bitCounter);
       //std::cout<<"/REWINDED "<<bitCounter<<", "<<curValue<<std::endl;
   }

    uint64_t nWordsRead() const
    {
        return (ptr - buffer) + ((bitCounter == BitStreamWordTypeNBits)? 0 : 1);
    }

	BitStreamWordType* buffer;

private:

	BitStreamWordType* ptr;
	BitStreamWordType curValue;
	size_t bitCounter;
};



class BitStreamWriter
{
public:
	BitStreamWriter(BitStreamWordType* buf) :
		ptr(buf), wordPos(0), counter(0)
	{
		*ptr = 0;
    }


	// todo this can be optimized.
	void putBits(MultiBitType v, unsigned NBits)
	{
		v &= ((1 << NBits) - 1); // people are allowed to send words with more bits, for convenience.
        if (wordPos + NBits < BitStreamWordTypeNBits)
        {
            // all fit in current word and 1 bit remains so no need to go to next word pos.
            //put it there, and update word position and done.
            (*ptr) |= (static_cast<BitStreamWordType>(v) << wordPos);
            wordPos += NBits;
        }
        else
        {
            // wordPos+1 most sign.bits still fit in. put them in, update NBits, and go to next word position.
            auto NBitsStillFit = BitStreamWordTypeNBits - wordPos;
            (*ptr) |= static_cast<BitStreamWordType>((v & ((1<<NBitsStillFit)-1))) << wordPos;
            v >>= NBitsStillFit;
            // go to next pos;
            ++ptr; ++counter;
            wordPos = NBits - NBitsStillFit;
            *ptr = v;
        }
	}

	size_t getWordSize()
	{
		size_t r = ((size_t)(counter));
		if (wordPos > 0) r+=1;
		return r;
	}

private:
	BitStreamWordType* ptr;
	size_t counter;
    BitStreamWordType wordPos;
};


// EerFrame.h
// Copyright (c) 2019 by FEI Company
#include <string>
#include <tiffio.h>
#include <vector>

namespace Fei {
namespace Acquisition {
namespace EerReader {


class EerFrame
{
public:
    EerFrame(TIFF* tiff);

    uint32_t GetWidth() const;
    uint32_t GetLength() const;
    uint16_t GetRleBits() const;
    uint16_t GetHorzSubBits() const;
    uint16_t GetVertSubBits() const;

    std::vector<unsigned char> GetEerData() const;
    int GetEncodingVersion() const;

private:
    uint32_t m_imageWidth;
    uint32_t m_imageLength;
    uint16_t m_rleBits;
    uint16_t m_horzSubBits;
    uint16_t m_vertSubBits;

    std::vector<unsigned char> m_eerData;
    int m_encodingVersion;
};

} //namespace EerReader
} //namespace Acquisition
} //namespace Fei

// Bitmap.h
// Copyright (c) 2019 by FEI Company
#include <string>
#include <tiffio.h>
#include <vector>

namespace Fei {
namespace Acquisition {
namespace EerReader {


class Bitmap
{
public:
    Bitmap(TIFF* tiff);

    uint32_t GetWidth() const;
    uint32_t GetLength() const;

    std::vector<unsigned char> GetImageData() const;
    int GetBitsPerSample() const;

private:
    uint32_t m_imageWidth;
    uint32_t m_imageLength;
    std::vector<unsigned char> m_imageData;
    uint16_t m_bitsPerSample;
};

} //namespace EerReader
} //namespace Acquisition
} //namespace Fei



// EerFile.h
// Copyright (c) 2019 by FEI Company
#include <memory>
#include <string>
#include <tiffio.h>

namespace Fei {
namespace Acquisition {
namespace EerReader {

class EerFile
{
public:
    EerFile(const std::string& filename);
    ~EerFile();

    std::unique_ptr<EerFrame> GetNextEerFrame();
    std::shared_ptr<Bitmap> GetFinalImage();
    std::string GetAcquisitionMetadata() const;

private:
    bool IsCurrentFrameEERCompressed();

private:
    std::shared_ptr<TIFF> m_tiff;
    bool m_nextFrameAvailable = true;
    std::shared_ptr<Bitmap> m_finalImageBitmap;
};

} //namespace EerReader
} //namespace Acquisition
} //namespace Fei


// Remainder of actual file

const bool g_no_bit_waste_on_overflow_code = true; // false for old prototype EER files.

struct ElectronPos
{
    uint16_t x;
    uint16_t y;
    
    ElectronPos(uint16_t x, uint16_t y) : x(x), y(y) {}
};

struct EerFrameSettings
{
    uint32_t width;
    uint32_t lenght;

    uint16_t rleBits;
    uint16_t horzSubBits;
    uint16_t vertSubBits;

    uint16_t bitsPerCode;
    uint16_t horizontalSubBitsOffset;
    uint16_t widthBitsOffset;

    explicit EerFrameSettings(const Fei::Acquisition::EerReader::EerFrame* eerFrame);

    // use for ECC mode
    EerFrameSettings();
};

class ElectronCountedFramesDecompressor
{
public:

	// constructor for read mode (gets sizes and options from file header)
    ElectronCountedFramesDecompressor(const std::string& filename);

    void getSize(unsigned& x, unsigned& y, unsigned& z);
    unsigned getNFrames();
    size_t getFileSize() { return m_fsizeBytes; }
    unsigned getNElectronsCounted() { return m_nElectronsCounted; }

    ///read entire image of specified size.
	void decompressImage(uint8_t* p, int superFactor=1, int frameNumber = -1);
    void decompressImage_AddTo(uint8_t* p, int superFactor=1, int frameNumber = -1);

	void decompressImage(float* p, int superFactor=1, int frameNumber = -1);
    void decompressImage_AddTo(float* p, int superFactor=1, int frameNumber = -1);

    //unsigned nElectronFrameUpperLimit();
    unsigned nElectronFractionUpperLimit(int frameStart, int frameStop);
    unsigned decompressCoordinateList(ElectronPos* pList, int frameNumber = -1);

    //void getNormalizedSubPixHist(float* r);


    ~ElectronCountedFramesDecompressor();


private:
    typedef uint64_t BitStreamWordType;

    void prepareRead();
    BitStreamer prepareFrameRead(int frameNumber = -1);
    void finalizeFrameRead(BitStreamer& myBitStreamer);
    void prepareCodec();

    void createIndex(); // creates the index on-the-fly for a headerless file.

    std::fstream m_fh;    // used in ECC mode
    std::unique_ptr<Fei::Acquisition::EerReader::EerFile> m_eerFile; // used in TIFF mode

	// for ecc
    std::vector<uint64_t> m_frameStartPointers;
	std::vector<BitStreamWordType> m_globalBuffer; // only used in headerless mode.
    
    // for tiff
    std::vector< std::vector<unsigned char> > m_frameBuffers; // only used in headerless mode.

    int m_frameCounter;

    EerFrameSettings m_eerFrameSettings;
    unsigned m_nFrames;
    size_t m_fsizeBytes;
    unsigned m_nElectronsCounted;

    bool m_tiffMode;

    float m_subPixCounts[16];

};
