/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/** \file     EncStage.h
    \brief
*/

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/Nal.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class PicShared
{
public:
  PicShared()
  : m_isSccWeak  ( false )
  , m_isSccStrong( false )
  , m_cts        ( 0 )
  , m_maxFrames  ( -1 )
  , m_poc        ( -1 )
  , m_refCount   ( -1 )
  , m_isLead     ( false )
  , m_isTrail    ( false )
  , m_ctsValid   ( false )
#if ENABLE_SPATIAL_SCALABLE
  , m_layerId    ( -1 )
  , m_ctuSize    ( -1 )
#endif
  {
    std::fill_n( m_prevShared, NUM_PREV_FRAMES, nullptr );
  };

  ~PicShared() {};

  bool         isUsed()          const { return m_refCount > 0; }
  void         incUsed()               { m_refCount += 1; }
  void         decUsed()               { CHECK( m_refCount <= 0, "release unused picture" ); if( m_refCount > 0 ) m_refCount -= 1; }
  bool         isLeadTrail()     const { return m_isLead || m_isTrail; }
  int          getPOC()          const { return m_poc; }
  ChromaFormat getChromaFormat() const { return m_origBuf.chromaFormat; }
  Size         getLumaSize()     const { return m_origBuf.Y(); }

#if ENABLE_SPATIAL_SCALABLE
  int          getLayerId()      const { return m_layerId; }
#endif
  void create( int maxFrames, ChromaFormat chromaFormat, const Size& size, bool useFilter )
  {
    CHECK( m_refCount >= 0, "PicShared already created" );

    m_maxFrames = maxFrames;
    m_refCount  = 0;

    const int padding = useFilter ? MCTF_PADDING : 0;
    m_origBuf.create( chromaFormat, Area( Position(), size ), 0, padding );
  }

#if ENABLE_SPATIAL_SCALABLE
  void reuse( int poc, const vvencYUVBuffer* yuvInBuf, int layerId)
#else
  void reuse( int poc, const vvencYUVBuffer* yuvInBuf )
#endif
  {
    CHECK( m_refCount < 0, "PicShared not created" );
    CHECK( isUsed(),       "PicShared still in use" );

    copyPadToPelUnitBuf( m_origBuf, *yuvInBuf, getChromaFormat() );

    m_isSccWeak   = false;
    m_isSccStrong = false;
    m_cts         = yuvInBuf->cts;
    m_poc         = poc;
    m_refCount    = 0;
    m_isLead      = poc < 0;
    m_isTrail     = m_maxFrames > 0 && poc >= m_maxFrames;
    m_ctsValid    = yuvInBuf->ctsValid;
    std::fill_n( m_prevShared, NUM_PREV_FRAMES, nullptr );
#if ENABLE_SPATIAL_SCALABLE
    m_layerId     = layerId;
#endif
  }

  void shareData( Picture* pic )
  {
    PelStorage* prevOrigBufs[ NUM_PREV_FRAMES ];
    for( int i = 0; i < NUM_PREV_FRAMES; i++ )
    {
      prevOrigBufs[ i ] = sharePrevOrigBuffer( i );
    }
    pic->linkSharedBuffers( &m_origBuf, &m_filteredBuf, prevOrigBufs, this );
    pic->isSccWeak   = m_isSccWeak;
    pic->isSccStrong = m_isSccStrong;
    pic->poc         = m_poc;
    pic->cts         = m_cts;
    pic->ctsValid    = m_ctsValid;
    m_refCount      += 1;
#if ENABLE_SPATIAL_SCALABLE
    pic->layerId     = m_layerId;
#endif
  }

  void releaseShared( Picture* pic )
  {
    pic->releaseSharedBuffers();
    for( int i = 0; i < NUM_PREV_FRAMES; i++ )
    {
      releasePrevOrigBuffer( i );
    }
    m_refCount -= 1;
    CHECK( m_refCount < 0, "PicShared invalid state" );
  };

private:
  PelStorage* sharePrevOrigBuffer( int idx )
  {
    CHECK( idx >= NUM_PREV_FRAMES, "array access out of bounds" );
    if( m_prevShared[ idx ] )
    {
      m_prevShared[ idx ]->m_refCount += 1;
      return &( m_prevShared[ idx ]->m_origBuf );
    }
    return nullptr;
  }

  void releasePrevOrigBuffer( int idx )
  {
    CHECK( idx >= NUM_PREV_FRAMES, "array access out of bounds" );
    if( m_prevShared[ idx ] )
    {
      m_prevShared[ idx ]->m_refCount -= 1;
      CHECK( m_refCount < 0, "PicShared invalid state" );
    }
  }

public:
  PicShared* m_prevShared[ NUM_PREV_FRAMES ];
  bool       m_isSccWeak;
  bool       m_isSccStrong;

private:
  PelStorage m_origBuf;
  PelStorage m_filteredBuf;
  uint64_t   m_cts;
  int        m_maxFrames;
  int        m_poc;
  int        m_refCount;
  bool       m_isLead;
  bool       m_isTrail;
  bool       m_ctsValid;
#if ENABLE_SPATIAL_SCALABLE
  int        m_layerId;
  int        m_ctuSize;
#endif
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class EncStage
{
public:
  EncStage()
  : m_nextStage       ( nullptr )
#if ENABLE_SPATIAL_SCALABLE
  , m_procList        ( nullptr )
#endif
  , m_minQueueSize    ( 0 )
  , m_flushAll        ( false )
  , m_processLeadTrail( false )
  , m_ctuSize         ( MAX_CU_SIZE )
  , m_isNonBlocking   ( false )
#if ENABLE_SPATIAL_SCALABLE
  , m_shared          ( false )
#endif
  , m_picCount        ( 0 )
  {
  };

  virtual ~EncStage()
  {
    freePicList();
#if ENABLE_SPATIAL_SCALABLE
    if ( !m_shared ) {
      delete m_procList;
    }
#endif
  };

  void freePicList()
  {
#if ENABLE_SPATIAL_SCALABLE
    for( auto pic : *m_procList )
#else
    for( auto pic : m_procList )
#endif
    {
      pic->destroy( true );
      delete pic;
    }
#if ENABLE_SPATIAL_SCALABLE
    m_procList->clear();
#else
    m_procList.clear();
#endif
    for( auto pic : m_freeList )
    {
      pic->destroy( true );
      delete pic;
    }
    m_freeList.clear();
  }

#if ENABLE_SPATIAL_SCALABLE
  bool isStageDone() const { return m_procList->empty(); }
#else
  bool isStageDone() const { return m_procList.empty(); }
#endif

#if ENABLE_SPATIAL_SCALABLE
  void initStage( int minQueueSize, bool flushAll, bool processLeadTrail, int ctuSize, bool nonBlocking = false, PicList* sharedPicList = nullptr )
#else
  void initStage( int minQueueSize, bool flushAll, bool processLeadTrail, int ctuSize, bool nonBlocking = false )
#endif
  {
    m_minQueueSize     = minQueueSize;
    m_flushAll         = flushAll;
    m_processLeadTrail = processLeadTrail;
    m_ctuSize          = ctuSize;
    m_isNonBlocking    = nonBlocking;
#if ENABLE_SPATIAL_SCALABLE
    if (sharedPicList) {
      m_procList = sharedPicList;
      m_shared = true;
    }
    else {
      m_procList = new PicList();
      m_shared = false;
    }
#endif
  }

  void linkNextStage( EncStage* nextStage )
  {
    m_nextStage = nextStage;
  }

  void addPicSorted( PicShared* picShared )
  {
    // send lead trail data to next stage if not requested
    if( ! m_processLeadTrail && picShared->isLeadTrail() )
    {
      if( m_nextStage )
      {
        m_nextStage->addPicSorted( picShared );
      }
      return;
    }

    // setup new picture or recycle old one
    const ChromaFormat chromaFormat = picShared->getChromaFormat();
    const Size lumaSize             = picShared->getLumaSize();
    Picture* pic                    = nullptr;
    if( m_freeList.size() )
    {
      pic = m_freeList.front();
      m_freeList.pop_front();
    }
    else
    {
      pic = new Picture();
      pic->create( chromaFormat, lumaSize, m_ctuSize, m_ctuSize + 16, false );
    }
    CHECK( pic == nullptr, "out of memory" );
    CHECK( pic->chromaFormat != chromaFormat || pic->Y().size() != lumaSize, "resolution or format changed" );

    pic->reset();
    picShared->shareData( pic );

    // call first picture init
    initPicture( pic );

    // sort picture into processing queue
    PicList::iterator picItr;
#if ENABLE_SPATIAL_SCALABLE
    for( picItr = m_procList->begin(); picItr != m_procList->end(); picItr++ )
#else
    for( picItr = m_procList.begin(); picItr != m_procList.end(); picItr++ )
#endif
    {
      if( pic->poc < ( *picItr )->poc )
        break;
    }
#if ENABLE_SPATIAL_SCALABLE
    m_procList->insert( picItr, pic );
#else
    m_procList.insert( picItr, pic );
#endif
    m_picCount++;
  }

  void runStage( bool flush, AccessUnitList& auList )
  {
    // ready to go?
#if ENABLE_SPATIAL_SCALABLE
    if( ( (int)m_procList->size() >= m_minQueueSize)
        || ( m_procList->size() && flush ) )
#else
    if( ( (int)m_procList.size() >= m_minQueueSize )
        || ( m_procList.size() && flush ) )
#endif
    {
      // process always one picture or all if encoder should be flushed
      do
      {
        // process pictures
        PicList doneList;
        PicList freeList;
#if ENABLE_SPATIAL_SCALABLE
        processPictures( *m_procList, flush, auList, doneList, freeList );
#else
        processPictures( m_procList, flush, auList, doneList, freeList );
#endif

        // send processed/finalized pictures to next stage
        if( m_nextStage )
        {
          for( auto pic : doneList )
          {
            m_nextStage->addPicSorted( pic->m_picShared );
          }
        }

        // release unused pictures
        for( auto pic : freeList )
        {
          // release shared buffer
          PicShared* picShared = pic->m_picShared;
          picShared->releaseShared( pic );
          // remove pic from own processing queue
#if ENABLE_SPATIAL_SCALABLE
          m_procList->remove( pic );
#else
          m_procList.remove( pic );
#endif
          m_freeList.push_back( pic );
        }
#if ENABLE_SPATIAL_SCALABLE
      } while (0);
#else
      } while( m_flushAll && flush && m_procList.size() );
#endif
    }
  }

  bool         isNonBlocking()     { return m_isNonBlocking; }
  virtual void waitForFreeEncoders()  {}
protected:
  virtual void initPicture    ( Picture* pic ) = 0;
  virtual void processPictures( const PicList& picList, bool flush, AccessUnitList& auList, PicList& doneList, PicList& freeList ) = 0;
private:
  EncStage* m_nextStage;
#if ENABLE_SPATIAL_SCALABLE
  PicList*  m_procList;
#else
  PicList   m_procList;
#endif
  PicList   m_freeList;
  int       m_minQueueSize;
  bool      m_flushAll;
  bool      m_processLeadTrail;
  int       m_ctuSize;
  bool      m_isNonBlocking;
#if ENABLE_SPATIAL_SCALABLE
  bool      m_shared;
#endif
protected:
  int64_t   m_picCount;
};

} // namespace vvenc

//! \}

