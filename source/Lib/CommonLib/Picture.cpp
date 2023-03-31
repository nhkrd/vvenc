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


/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "SEI.h"

#if ENABLE_SPATIAL_SCALABLE
#include "InterpolationFilter.h"
#include "CommonDef.h"
#endif

#include <math.h>

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ---------------------------------------------------------------------------
// picture methods
// ---------------------------------------------------------------------------




// ====================================================================================================================
// AMaxBT block statistics
// ====================================================================================================================


void BlkStat::storeBlkSize( const Picture& pic )
{
  const Slice& slice = *(pic.slices[ 0 ]);

  ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
  ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );

  if ( ! slice.isIRAP() )
  {
    const int refLayer = slice.depth < NUM_AMAXBT_LAYER ? slice.depth: NUM_AMAXBT_LAYER - 1;
    for ( const CodingUnit *cu : pic.cs->cus )
    {
      m_uiBlkSize[ refLayer ] += cu->Y().area();
      m_uiNumBlk [ refLayer ] += 1;
    }
  }
}

void BlkStat::updateMaxBT( const Slice& slice, const BlkStat& blkStat )
{
  if ( ! slice.isIRAP() )
  {
    const int refLayer = slice.depth < NUM_AMAXBT_LAYER ? slice.depth: NUM_AMAXBT_LAYER - 1;
    m_uiBlkSize[ refLayer ] += blkStat.m_uiBlkSize[ refLayer ];
    m_uiNumBlk [ refLayer ] += blkStat.m_uiNumBlk [ refLayer ];
  }
}

void BlkStat::setSliceMaxBT( Slice& slice )
{
  if( ! slice.isIRAP() )
  {
    int refLayer = slice.depth < NUM_AMAXBT_LAYER ? slice.depth: NUM_AMAXBT_LAYER - 1;
    if( m_bResetAMaxBT && slice.poc > m_uiPrevISlicePOC )
    {
      ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
      ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
      m_bResetAMaxBT = false;
    }

    if( refLayer >= 0 && m_uiNumBlk[ refLayer ] != 0 )
    {
      slice.picHeader->splitConsOverride = true;
      double dBlkSize = sqrt( ( double ) m_uiBlkSize[refLayer] / m_uiNumBlk[refLayer] );
      if( dBlkSize < AMAXBT_TH32 || slice.sps->CTUSize == 32 )
      {
        slice.picHeader->maxBTSize[1] = ( 32 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 32 );
      }
      else if( dBlkSize < AMAXBT_TH64 || slice.sps->CTUSize == 64 )
      {
        slice.picHeader->maxBTSize[1] = ( 64 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 64 );
      }
      else
      {
        slice.picHeader->maxBTSize[1] = ( 128 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 128 );
      }

      m_uiBlkSize[ refLayer ] = 0;
      m_uiNumBlk [ refLayer ] = 0;
    }
  }
  else
  {
    if( m_bResetAMaxBT )
    {
      ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
      ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
    }

    m_uiPrevISlicePOC = slice.poc;
    m_bResetAMaxBT    = true;
  }
}


// ====================================================================================================================
// Picture
// ====================================================================================================================


Picture::Picture()
    : cs                ( nullptr )
    , vps               ( nullptr )
    , dci               ( nullptr )
    , picApsMap         ( MAX_NUM_APS * MAX_NUM_APS_TYPE )
    , isInitDone        ( false )
    , isReconstructed   ( false )
    , isBorderExtended  ( false )
    , isReferenced      ( false )
    , isNeededForOutput ( false )
    , isFinished        ( false )
    , isLongTerm        ( false )
    , encPic            ( true )
    , writePic          ( true )
    , precedingDRAP     ( false )
    , refCounter        ( 0 )
    , poc               ( 0 )
    , gopId             ( 0 )
    , rcIdxInGop        ( 0 )
    , TLayer            ( std::numeric_limits<uint32_t>::max() )
    , layerId           ( 0 )
    , isSubPicBorderSaved (false)
    , sliceDataNumBins  ( 0 )
    , cts               ( 0 )
    , ctsValid          ( false )
    , isPreAnalysis     ( false )
    , m_picShared       ( nullptr )
    , picInitialQP      ( 0 )
    , picVisActTL0      ( 0 )
    , picVisActY        ( 0 )
    , isSccWeak         ( false )
    , isSccStrong       ( false )
    , useScME           ( false )
    , useScMCTF         ( false )
    , useScTS           ( false )
    , useScBDPCM        ( false )
    , useScIBC          ( false )
    , useScLMCS         ( false )
    , useQtbttSpeedUpMode( 0 )
    , seqBaseQp         ( 0 )
    , actualHeadBits    ( 0 )
    , actualTotalBits   ( 0 )
    , encRCPic          ( nullptr )
#if ENABLE_SPATIAL_SCALABLE
    , unscaledPic       ( nullptr )
    , shrdUnitCache     ( nullptr )
#endif
{
  std::fill_n( m_sharedBufs, (int)NUM_PIC_TYPES, nullptr );
  std::fill_n( m_bufsOrigPrev, NUM_PREV_FRAMES, nullptr );
}

void Picture::create( ChromaFormat _chromaFormat, const Size& size, unsigned _maxCUSize, unsigned _margin, bool _decoder )
{
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            =  _margin;

  if( _decoder )
  {
    m_picBufs[ PIC_RESIDUAL   ].create( _chromaFormat, Area( 0, 0, _maxCUSize, _maxCUSize ) );
    m_picBufs[ PIC_PREDICTION ].create( _chromaFormat, Area( 0, 0, _maxCUSize, _maxCUSize ) );
  }
}

void Picture::reset()
{
  // reset picture
  isInitDone          = false;
  isReconstructed     = false;
  isBorderExtended    = false;
  isReferenced        = true;
  isNeededForOutput   = true;
  isFinished          = false;
  isLongTerm          = false;
  encPic              = false;
  writePic            = false;
  precedingDRAP       = false;

  refCounter          = 0;
  poc                 = -1;
  gopId               = 0;
  rcIdxInGop          = 0;
  TLayer              = std::numeric_limits<uint32_t>::max();

  actualHeadBits      = 0;
  actualTotalBits     = 0;

  std::fill_n( m_sharedBufs, (int)NUM_PIC_TYPES, nullptr );
  std::fill_n( m_bufsOrigPrev, NUM_PREV_FRAMES, nullptr );

  encTime.resetTimer();
}

void Picture::destroy( bool bPicHeader )
{
  for (uint32_t t = 0; t < NUM_PIC_TYPES; t++)
  {
    m_picBufs[  t ].destroy();
  }
  if( cs )
  {
    if( bPicHeader && cs->picHeader )
    {
      delete cs->picHeader;
    }
    cs->picHeader = nullptr;
    cs->destroy();
    delete cs;
    cs = nullptr;
  }

  for( auto &ps : slices )
  {
    delete ps;
  }
  slices.clear();

  for( auto &psei : SEIs )
  {
    delete psei;
  }
  SEIs.clear();
}

void Picture::linkSharedBuffers( PelStorage* origBuf, PelStorage* filteredBuf, PelStorage* prevOrigBufs[ NUM_PREV_FRAMES ], PicShared* picShared )
{
  m_picShared                      = picShared;
  m_sharedBufs[ PIC_ORIGINAL ]     = origBuf;
  m_sharedBufs[ PIC_ORIGINAL_RSP ] = filteredBuf;
  for( int i = 0; i < NUM_PREV_FRAMES; i++ )
    m_bufsOrigPrev[ i ] = prevOrigBufs[ i ];
}

void Picture::releaseSharedBuffers()
{
  m_picShared                      = nullptr;
  m_sharedBufs[ PIC_ORIGINAL ]     = nullptr;
  m_sharedBufs[ PIC_ORIGINAL_RSP ] = nullptr;
}

void Picture::createTempBuffers( unsigned _maxCUSize )
{
  CHECK( !cs, "Coding structure is required a this point!" );

  m_picBufs[PIC_SAO_TEMP].create( chromaFormat, Y(), cs->pcv->maxCUSize, 0, MEMORY_ALIGN_DEF_SIZE );

  if( cs ) cs->rebindPicBufs();
}

void Picture::destroyTempBuffers()
{
  m_picBufs[PIC_SAO_TEMP].destroy();

  if( cs ) cs->rebindPicBufs();
}

const CPelBuf     Picture::getOrigBufPrev (const CompArea &blk, const PrevFrameType type) const { return (m_bufsOrigPrev[ type ] && blk.valid() ? m_bufsOrigPrev[ type ]->getBuf (blk) : PelBuf()); }
const CPelUnitBuf Picture::getOrigBufPrev (const PrevFrameType type) const { return (m_bufsOrigPrev[ type ] ? *m_bufsOrigPrev[ type ] : PelUnitBuf()); }
const CPelBuf     Picture::getOrigBufPrev (const ComponentID compID, const PrevFrameType type) const { return (m_bufsOrigPrev[ type ] ? m_bufsOrigPrev[ type ]->getBuf (compID) : PelBuf()); }

void Picture::finalInit( const VPS& _vps, const SPS& sps, const PPS& pps, PicHeader* picHeader, XUCache& unitCache, std::mutex* mutex, APS** alfAps, APS* lmcsAps )
{
  for( auto &sei : SEIs )
  {
    delete sei;
  }
  SEIs.clear();

  for (size_t i = 0; i < slices.size(); i++)
  {
    delete slices[i];
  }
  slices.clear();

  const ChromaFormat chromaFormatIDC = sps.chromaFormatIdc;
  const int          iWidth = pps.picWidthInLumaSamples;
  const int          iHeight = pps.picHeightInLumaSamples;

  if( cs )
  {
    CHECK( cs->sps != &sps,  "picture initialization error: sps changed" );
    CHECK( cs->vps != &_vps, "picture initialization error: vps changed" );
  }
  else
  {
    cs = new CodingStructure( unitCache, mutex );
    cs->pps = &pps;
    cs->sps = &sps;
    cs->vps = &_vps;
    cs->create( UnitArea( chromaFormatIDC, Area( 0, 0, iWidth, iHeight )), true, pps.pcv );
  }

  cs->picture   = this;
  cs->refCS     = cs;
  cs->slice     = nullptr;  // the slices for this picture have not been set at this point. update cs->slice after swapSliceObject()
  cs->picHeader = picHeader;
  if ( alfAps )
  {
    memcpy(cs->alfAps, alfAps, sizeof(cs->alfAps));
  }
  cs->lmcsAps = lmcsAps;
  cs->pcv     = pps.pcv;
  vps         = &_vps;
  dci         = nullptr;

  if( ! m_picBufs[ PIC_RECONSTRUCTION ].valid() )
  {
    m_picBufs[ PIC_RECONSTRUCTION ].create( chromaFormat, Area( lumaPos(), lumaSize() ), sps.CTUSize, margin, MEMORY_ALIGN_DEF_SIZE );
  }

  sliceDataStreams.clear();
  sliceDataNumBins = 0;

#if ENABLE_SPATIAL_SCALABLE
  shrdUnitCache = &unitCache;
  unitChacheMutex = mutex;
#endif
}

void Picture::setSccFlags( const VVEncCfg* encCfg )
{
  useScME    = encCfg->m_motionEstimationSearchMethodSCC > 0                          && isSccStrong;
  useScTS    = encCfg->m_TS == 1                || ( encCfg->m_TS == 2                && isSccWeak );
  useScBDPCM = encCfg->m_useBDPCM == 1          || ( encCfg->m_useBDPCM == 2          && isSccWeak );
  useScMCTF  = encCfg->m_vvencMCTF.MCTF == 1    || ( encCfg->m_vvencMCTF.MCTF == 2    && ! isSccStrong );
  useScLMCS  = encCfg->m_lumaReshapeEnable == 1 || ( encCfg->m_lumaReshapeEnable == 2 && ! isSccStrong );
  useScIBC   = encCfg->m_IBCMode == 1           || ( encCfg->m_IBCMode == 2           && isSccStrong );
  useQtbttSpeedUpMode = encCfg->m_qtbttSpeedUpMode;

  if( ( encCfg->m_qtbttSpeedUpMode & 2 ) && isSccStrong )
  {
    useQtbttSpeedUpMode &= ~1;
  }
}

Slice* Picture::allocateNewSlice()
{
  slices.push_back( new Slice );
  Slice& slice = *slices.back();

  slice.pic     = this;
  slice.pps     = cs->pps;
  slice.sps     = cs->sps;
  slice.vps     = cs->vps;

  memcpy( slice.alfAps, cs->alfAps, sizeof(cs->alfAps) );

  if ( slices.size() >= 2 )
  {
    slice.copySliceInfo( slices[ slices.size() - 2 ] );
  }

  return slices.back();
}

Slice* Picture::swapSliceObject( Slice*  p, uint32_t i )
{
  p->pic     = this;
  p->pps     = cs->pps;
  p->sps     = cs->sps;
  p->vps     = cs->vps;
  memcpy( p->alfAps, cs->alfAps, sizeof(cs->alfAps) );

  Slice*  pTmp = slices[ i ];
  slices[ i ] = p;

  pTmp->pic = ( nullptr );
  pTmp->sps = ( nullptr );
  pTmp->pps = ( nullptr );
  memset( pTmp->alfAps, 0, sizeof( *pTmp->alfAps ) * ALF_CTB_MAX_NUM_APS );

  return pTmp;
}

void Picture::extendPicBorder()
{
  if ( isBorderExtended )
  {
    return;
  }

  for(int comp=0; comp<getNumberValidComponents( cs->area.chromaFormat ); comp++)
  {
    ComponentID compID = ComponentID( comp );
    PelBuf p = m_picBufs[ PIC_RECONSTRUCTION ].get( compID );
    Pel* piTxt = p.bufAt(0,0);
    int xmargin = margin >> getComponentScaleX( compID, cs->area.chromaFormat );
    int ymargin = margin >> getComponentScaleY( compID, cs->area.chromaFormat );

    Pel*  pi = piTxt;
    // do left and right margins
      for (int y = 0; y < p.height; y++)
      {
        for (int x = 0; x < xmargin; x++ )
        {
          pi[ -xmargin + x ] = pi[0];
          pi[  p.width + x ] = pi[p.width-1];
        }
        pi += p.stride;
      }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (p.stride + xmargin);
    // pi is now the (-marginX, height-1)
    for (int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi + (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin << 1)));
    }

    // pi is still (-marginX, height-1)
    pi -= ((p.height-1) * p.stride);
    // pi is now (-marginX, 0)
    for (int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi - (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin<<1)) );
    }

    // reference picture with horizontal wrapped boundary
    if (cs->sps->wrapAroundEnabled)
    {
      p = m_picBufs[ PIC_RECON_WRAP ].get( compID );
      p.copyFrom(m_picBufs[ PIC_RECONSTRUCTION ].get( compID ));
      piTxt = p.bufAt(0,0);
      pi = piTxt;
      int xoffset = cs->pps->wrapAroundOffset >> getComponentScaleX( compID, cs->area.chromaFormat );
      for (int y = 0; y < p.height; y++)
      {
        for (int x = 0; x < xmargin; x++ )
        {
          if( x < xoffset )
          {
            pi[ -x - 1 ] = pi[ -x - 1 + xoffset ];
            pi[  p.width + x ] = pi[ p.width + x - xoffset ];
          }
          else
          {
            pi[ -x - 1 ] = pi[ 0 ];
            pi[  p.width + x ] = pi[ p.width - 1 ];
          }
        }
        pi += p.stride;
      }
      pi -= (p.stride + xmargin);
      for (int y = 0; y < ymargin; y++ )
      {
        ::memcpy( pi + (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin << 1)));
      }
      pi -= ((p.height-1) * p.stride);
      for (int y = 0; y < ymargin; y++ )
      {
        ::memcpy( pi - (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin<<1)) );
      }
    }
  }

  isBorderExtended = true;
}

PelUnitBuf Picture::getPicBuf( const UnitArea& unit, const PictureType type )
{
  if( chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( chromaFormat, getPicBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( chromaFormat, getPicBuf( unit.Y(), type ), getPicBuf( unit.Cb(), type ), getPicBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf Picture::getPicBuf( const UnitArea& unit, const PictureType type ) const
{
  if( chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( chromaFormat, getPicBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( chromaFormat, getPicBuf( unit.Y(), type ), getPicBuf( unit.Cb(), type ), getPicBuf( unit.Cr(), type ) );
  }
}

PelUnitBuf Picture::getSharedBuf( const UnitArea& unit, const PictureType type )
{
  if( chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( chromaFormat, getSharedBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( chromaFormat, getSharedBuf( unit.Y(), type ), getSharedBuf( unit.Cb(), type ), getSharedBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf Picture::getSharedBuf( const UnitArea& unit, const PictureType type ) const
{
  if( chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( chromaFormat, getSharedBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( chromaFormat, getSharedBuf( unit.Y(), type ), getSharedBuf( unit.Cb(), type ), getSharedBuf( unit.Cr(), type ) );
  }
}

void Picture::resizeAlfCtuBuffers( int numEntries )
{
  for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    m_alfCtuEnabled[compIdx].resize( numEntries );
    std::fill( m_alfCtuEnabled[compIdx].begin(), m_alfCtuEnabled[compIdx].end(), 0 );
  }

  m_alfCtbFilterIndex.resize(numEntries);
  for (int i = 0; i < numEntries; i++)
  {
    m_alfCtbFilterIndex[i] = 0;
  }

  for( int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    m_alfCtuAlternative[compIdx].resize( numEntries );
    std::fill( m_alfCtuAlternative[compIdx].begin(), m_alfCtuAlternative[compIdx].end(), 0 );
  }
}

#if ENABLE_SPATIAL_SCALABLE
const TFilterCoeff DownsamplingFilterSRC[8][16][12] =
{
    { // D = 1
      {   0,   0,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0 },
      {   0,   0,   0,   2,  -6, 127,   7,  -2,   0,   0,   0,   0 },
      {   0,   0,   0,   3, -12, 125,  16,  -5,   1,   0,   0,   0 },
      {   0,   0,   0,   4, -16, 120,  26,  -7,   1,   0,   0,   0 },
      {   0,   0,   0,   5, -18, 114,  36, -10,   1,   0,   0,   0 },
      {   0,   0,   0,   5, -20, 107,  46, -12,   2,   0,   0,   0 },
      {   0,   0,   0,   5, -21,  99,  57, -15,   3,   0,   0,   0 },
      {   0,   0,   0,   5, -20,  89,  68, -18,   4,   0,   0,   0 },
      {   0,   0,   0,   4, -19,  79,  79, -19,   4,   0,   0,   0 },
      {   0,   0,   0,   4, -18,  68,  89, -20,   5,   0,   0,   0 },
      {   0,   0,   0,   3, -15,  57,  99, -21,   5,   0,   0,   0 },
      {   0,   0,   0,   2, -12,  46, 107, -20,   5,   0,   0,   0 },
      {   0,   0,   0,   1, -10,  36, 114, -18,   5,   0,   0,   0 },
      {   0,   0,   0,   1,  -7,  26, 120, -16,   4,   0,   0,   0 },
      {   0,   0,   0,   1,  -5,  16, 125, -12,   3,   0,   0,   0 },
      {   0,   0,   0,   0,  -2,   7, 127,  -6,   2,   0,   0,   0 }
    },
    { // D = 1.5
      {   0,   2,   0, -14,  33,  86,  33, -14,   0,   2,   0,   0 },
      {   0,   1,   1, -14,  29,  85,  38, -13,  -1,   2,   0,   0 },
      {   0,   1,   2, -14,  24,  84,  43, -12,  -2,   2,   0,   0 },
      {   0,   1,   2, -13,  19,  83,  48, -11,  -3,   2,   0,   0 },
      {   0,   0,   3, -13,  15,  81,  53, -10,  -4,   3,   0,   0 },
      {   0,   0,   3, -12,  11,  79,  57,  -8,  -5,   3,   0,   0 },
      {   0,   0,   3, -11,   7,  76,  62,  -5,  -7,   3,   0,   0 },
      {   0,   0,   3, -10,   3,  73,  65,  -2,  -7,   3,   0,   0 },
      {   0,   0,   3,  -9,   0,  70,  70,   0,  -9,   3,   0,   0 },
      {   0,   0,   3,  -7,  -2,  65,  73,   3, -10,   3,   0,   0 },
      {   0,   0,   3,  -7,  -5,  62,  76,   7, -11,   3,   0,   0 },
      {   0,   0,   3,  -5,  -8,  57,  79,  11, -12,   3,   0,   0 },
      {   0,   0,   3,  -4, -10,  53,  81,  15, -13,   3,   0,   0 },
      {   0,   0,   2,  -3, -11,  48,  83,  19, -13,   2,   1,   0 },
      {   0,   0,   2,  -2, -12,  43,  84,  24, -14,   2,   1,   0 },
      {   0,   0,   2,  -1, -13,  38,  85,  29, -14,   1,   1,   0 }
    },
    { // D = 2
      {   0,   5,   -6,  -10,  37,  76,   37,  -10,  -6,    5,  0,   0}, //0
      {   0,   5,   -4,  -11,  33,  76,   40,  -9,    -7,    5,  0,   0}, //1
      //{   0,   5,   -3,  -12,  28,  75,   44,  -7,    -8,    5,  1,   0}, //2
      {  -1,   5,   -3,  -12,  29,  75,   45,  -7,    -8,   5,  0,   0}, //2 new coefficients in m24499
      {  -1,   4,   -2,  -13,  25,  75,   48,  -5,    -9,    5,  1,   0}, //3
      {  -1,   4,   -1,  -13,  22,  73,   52,  -3,    -10,  4,  1,   0}, //4
      {  -1,   4,   0,    -13,  18,  72,   55,  -1,    -11,  4,  2,  -1}, //5
      {  -1,   4,   1,    -13,  14,  70,   59,  2,    -12,  3,  2,  -1}, //6
      {  -1,   3,   1,    -13,  11,  68,   62,  5,    -12,  3,  2,  -1}, //7
      {  -1,   3,   2,    -13,  8,  65,   65,  8,    -13,  2,  3,  -1}, //8
      {  -1,   2,   3,    -12,  5,  62,   68,  11,    -13,  1,  3,  -1}, //9
      {  -1,   2,   3,    -12,  2,  59,   70,  14,    -13,  1,  4,  -1}, //10
      {  -1,   2,   4,    -11,  -1,  55,   72,  18,    -13,  0,  4,  -1}, //11
      {   0,   1,   4,    -10,  -3,  52,   73,  22,    -13,  -1,  4,  -1}, //12
      {   0,   1,   5,    -9,    -5,  48,   75,  25,    -13,  -2,  4,  -1}, //13
      //{   0,   1,   5,    -8,    -7,  44,   75,  28,    -12,  -3,  5,   0}, //14
      {    0,   0,   5,    -8,   -7,  45,   75,  29,    -12,  -3,  5,  -1}  , //14 new coefficients in m24499
      {   0,   0,   5,    -7,    -9,  40,   76,  33,    -11,  -4,  5,   0}, //15
    },
    { // D = 2.5
      {   2,  -3,   -9,  6,   39,  58,   39,  6,   -9,  -3,    2,    0}, // 0
      {   2,  -3,   -9,  4,   38,  58,   43,  7,   -9,  -4,    1,    0}, // 1
      {   2,  -2,   -9,  2,   35,  58,   44,  9,   -8,  -4,    1,    0}, // 2
      {   1,  -2,   -9,  1,   34,  58,   46,  11,   -8,  -5,    1,    0}, // 3
      //{   1,  -1,   -8,  -1,   31,  57,   48,  13,   -8,  -5,    1,    0}, // 4
      {   1,  -1,   -8,  -1,   31,  57,   47,  13,   -7,  -5,    1,    0},  // 4 new coefficients in m24499
      {   1,  -1,   -8,  -2,   29,  56,   49,  15,   -7,  -6,    1,    1}, // 5
      {   1,  0,   -8,  -3,   26,  55,   51,  17,   -7,  -6,    1,    1}, // 6
      {   1,  0,   -7,  -4,   24,  54,   52,  19,   -6,  -7,    1,    1}, // 7
      {   1,  0,   -7,  -5,   22,  53,   53,  22,   -5,  -7,    0,    1}, // 8
      {   1,  1,   -7,  -6,   19,  52,   54,  24,   -4,  -7,    0,    1}, // 9
      {   1,  1,   -6,  -7,   17,  51,   55,  26,   -3,  -8,    0,    1}, // 10
      {   1,  1,   -6,  -7,   15,  49,   56,  29,   -2,  -8,    -1,    1}, // 11
      //{   0,  1,   -5,  -8,   13,  48,   57,  31,   -1,  -8,    -1,    1}, // 12 new coefficients in m24499
      {   0,  1,   -5,  -7,   13,  47,  57,  31,  -1,    -8,   -1,    1}, // 12
      {   0,  1,   -5,  -8,   11,  46,   58,  34,   1,    -9,    -2,    1}, // 13
      {   0,  1,   -4,  -8,   9,    44,   58,  35,   2,    -9,    -2,    2}, // 14
      {   0,  1,   -4,  -9,   7,    43,   58,  38,   4,    -9,    -3,    2}, // 15
    },
    { // D = 3
      {  -2,  -7,   0,  17,  35,  43,  35,  17,   0,  -7,  -5,   2 },
      {  -2,  -7,  -1,  16,  34,  43,  36,  18,   1,  -7,  -5,   2 },
      {  -1,  -7,  -1,  14,  33,  43,  36,  19,   1,  -6,  -5,   2 },
      {  -1,  -7,  -2,  13,  32,  42,  37,  20,   3,  -6,  -5,   2 },
      {   0,  -7,  -3,  12,  31,  42,  38,  21,   3,  -6,  -5,   2 },
      {   0,  -7,  -3,  11,  30,  42,  39,  23,   4,  -6,  -6,   1 },
      {   0,  -7,  -4,  10,  29,  42,  40,  24,   5,  -6,  -6,   1 },
      {   1,  -7,  -4,   9,  27,  41,  40,  25,   6,  -5,  -6,   1 },
      {   1,  -6,  -5,   7,  26,  41,  41,  26,   7,  -5,  -6,   1 },
      {   1,  -6,  -5,   6,  25,  40,  41,  27,   9,  -4,  -7,   1 },
      {   1,  -6,  -6,   5,  24,  40,  42,  29,  10,  -4,  -7,   0 },
      {   1,  -6,  -6,   4,  23,  39,  42,  30,  11,  -3,  -7,   0 },
      {   2,  -5,  -6,   3,  21,  38,  42,  31,  12,  -3,  -7,   0 },
      {   2,  -5,  -6,   3,  20,  37,  42,  32,  13,  -2,  -7,  -1 },
      {   2,  -5,  -6,   1,  19,  36,  43,  33,  14,  -1,  -7,  -1 },
      {   2,  -5,  -7,   1,  18,  36,  43,  34,  16,  -1,  -7,  -2 }
    },
    { // D = 3.5
      {  -6,  -3,   5,  19,  31,  36,  31,  19,   5,  -3,  -6,   0 },
      {  -6,  -4,   4,  18,  31,  37,  32,  20,   6,  -3,  -6,  -1 },
      {  -6,  -4,   4,  17,  30,  36,  33,  21,   7,  -3,  -6,  -1 },
      {  -5,  -5,   3,  16,  30,  36,  33,  22,   8,  -2,  -6,  -2 },
      {  -5,  -5,   2,  15,  29,  36,  34,  23,   9,  -2,  -6,  -2 },
      {  -5,  -5,   2,  15,  28,  36,  34,  24,  10,  -2,  -6,  -3 },
      {  -4,  -5,   1,  14,  27,  36,  35,  24,  10,  -1,  -6,  -3 },
      {  -4,  -5,   0,  13,  26,  35,  35,  25,  11,   0,  -5,  -3 },
      {  -4,  -6,   0,  12,  26,  36,  36,  26,  12,   0,  -6,  -4 },
      {  -3,  -5,   0,  11,  25,  35,  35,  26,  13,   0,  -5,  -4 },
      {  -3,  -6,  -1,  10,  24,  35,  36,  27,  14,   1,  -5,  -4 },
      {  -3,  -6,  -2,  10,  24,  34,  36,  28,  15,   2,  -5,  -5 },
      {  -2,  -6,  -2,   9,  23,  34,  36,  29,  15,   2,  -5,  -5 },
      {  -2,  -6,  -2,   8,  22,  33,  36,  30,  16,   3,  -5,  -5 },
      {  -1,  -6,  -3,   7,  21,  33,  36,  30,  17,   4,  -4,  -6 },
      {  -1,  -6,  -3,   6,  20,  32,  37,  31,  18,   4,  -4,  -6 }
    },
    { // D = 4
      {  -9,   0,   9,  20,  28,  32,  28,  20,   9,   0,  -9,   0 },
      {  -9,   0,   8,  19,  28,  32,  29,  20,  10,   0,  -4,  -5 },
      {  -9,  -1,   8,  18,  28,  32,  29,  21,  10,   1,  -4,  -5 },
      {  -9,  -1,   7,  18,  27,  32,  30,  22,  11,   1,  -4,  -6 },
      {  -8,  -2,   6,  17,  27,  32,  30,  22,  12,   2,  -4,  -6 },
      {  -8,  -2,   6,  16,  26,  32,  31,  23,  12,   2,  -4,  -6 },
      {  -8,  -2,   5,  16,  26,  31,  31,  23,  13,   3,  -3,  -7 },
      {  -8,  -3,   5,  15,  25,  31,  31,  24,  14,   4,  -3,  -7 },
      {  -7,  -3,   4,  14,  25,  31,  31,  25,  14,   4,  -3,  -7 },
      {  -7,  -3,   4,  14,  24,  31,  31,  25,  15,   5,  -3,  -8 },
      {  -7,  -3,   3,  13,  23,  31,  31,  26,  16,   5,  -2,  -8 },
      {  -6,  -4,   2,  12,  23,  31,  32,  26,  16,   6,  -2,  -8 },
      {  -6,  -4,   2,  12,  22,  30,  32,  27,  17,   6,  -2,  -8 },
      {  -6,  -4,   1,  11,  22,  30,  32,  27,  18,   7,  -1,  -9 },
      {  -5,  -4,   1,  10,  21,  29,  32,  28,  18,   8,  -1,  -9 },
      {  -5,  -4,   0,  10,  20,  29,  32,  28,  19,   8,   0,  -9 }
    },
    { // D = 5.5
      {  -8,   7,  13,  18,  22,  24,  22,  18,  13,   7,   2, -10 },
      {  -8,   7,  13,  18,  22,  23,  22,  19,  13,   7,   2, -10 },
      {  -8,   6,  12,  18,  22,  23,  22,  19,  14,   8,   2, -10 },
      {  -9,   6,  12,  17,  22,  23,  23,  19,  14,   8,   3, -10 },
      {  -9,   6,  12,  17,  21,  23,  23,  19,  14,   9,   3, -10 },
      {  -9,   5,  11,  17,  21,  23,  23,  20,  15,   9,   3, -10 },
      {  -9,   5,  11,  16,  21,  23,  23,  20,  15,   9,   4, -10 },
      {  -9,   5,  10,  16,  21,  23,  23,  20,  15,  10,   4, -10 },
      { -10,   5,  10,  16,  20,  23,  23,  20,  16,  10,   5, -10 },
      { -10,   4,  10,  15,  20,  23,  23,  21,  16,  10,   5,  -9 },
      { -10,   4,   9,  15,  20,  23,  23,  21,  16,  11,   5,  -9 },
      { -10,   3,   9,  15,  20,  23,  23,  21,  17,  11,   5,  -9 },
      { -10,   3,   9,  14,  19,  23,  23,  21,  17,  12,   6,  -9 },
      { -10,   3,   8,  14,  19,  23,  23,  22,  17,  12,   6,  -9 },
      { -10,   2,   8,  14,  19,  22,  23,  22,  18,  12,   6,  -8 },
      { -10,   2,   7,  13,  19,  22,  23,  22,  18,  13,   7,  -8 }
    }
};

void Picture::sampleRateConv(const std::pair<int, int> scalingRatio, const std::pair<int, int> compScale,
  const CPelBuf& beforeScale, const int beforeScaleLeftOffset, const int beforeScaleTopOffset,
  const PelBuf& afterScale, const int afterScaleLeftOffset, const int afterScaleTopOffset,
  const int bitDepth, const bool useLumaFilter, const bool downsampling,
  const bool horCollocatedPositionFlag, const bool verCollocatedPositionFlag)
{
  const Pel* orgSrc = beforeScale.buf;
  const int orgWidth = beforeScale.width;
  const int orgHeight = beforeScale.height;
  const int orgStride = beforeScale.stride;

  Pel* scaledSrc = afterScale.buf;
  const int scaledWidth = afterScale.width;
  const int scaledHeight = afterScale.height;
  const int scaledStride = afterScale.stride;

  if (orgWidth == scaledWidth && orgHeight == scaledHeight && scalingRatio == SCALE_1X && !beforeScaleLeftOffset && !beforeScaleTopOffset && !afterScaleLeftOffset && !afterScaleTopOffset)
  {
    for (int j = 0; j < orgHeight; j++)
    {
      memcpy(scaledSrc + j * scaledStride, orgSrc + j * orgStride, sizeof(Pel) * orgWidth);
    }

    return;
  }

  const TFilterCoeff* filterHor = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const TFilterCoeff* filterVer = useLumaFilter ? &InterpolationFilter::m_lumaFilter[0][0] : &InterpolationFilter::m_chromaFilter[0][0];
  const int numFracPositions = useLumaFilter ? 15 : 31;
  const int numFracShift = useLumaFilter ? 4 : 5;
  const int posShiftX = SCALE_RATIO_BITS - numFracShift + compScale.first;
  const int posShiftY = SCALE_RATIO_BITS - numFracShift + compScale.second;
  int addX = (1 << (posShiftX - 1)) + (beforeScaleLeftOffset << SCALE_RATIO_BITS) + ((int(1 - horCollocatedPositionFlag) * 8 * (scalingRatio.first - SCALE_1X.first) + (1 << (2 + compScale.first))) >> (3 + compScale.first));
  int addY = (1 << (posShiftY - 1)) + (beforeScaleTopOffset << SCALE_RATIO_BITS) + ((int(1 - verCollocatedPositionFlag) * 8 * (scalingRatio.second - SCALE_1X.second) + (1 << (2 + compScale.second))) >> (3 + compScale.second));

  if (downsampling)
  {
    int verFilter = 0;
    int horFilter = 0;

    if (scalingRatio.first > (15 << SCALE_RATIO_BITS) / 4)
    {
      horFilter = 7;
    }
    else if (scalingRatio.first > (20 << SCALE_RATIO_BITS) / 7)
    {
      horFilter = 6;
    }
    else if (scalingRatio.first > (5 << SCALE_RATIO_BITS) / 2)
    {
      horFilter = 5;
    }
    else if (scalingRatio.first > (2 << SCALE_RATIO_BITS))
    {
      horFilter = 4;
    }
    else if (scalingRatio.first > (5 << SCALE_RATIO_BITS) / 3)
    {
      horFilter = 3;
    }
    else if (scalingRatio.first > (5 << SCALE_RATIO_BITS) / 4)
    {
      horFilter = 2;
    }
    else if (scalingRatio.first > (20 << SCALE_RATIO_BITS) / 19)
    {
      horFilter = 1;
    }

    if (scalingRatio.second > (15 << SCALE_RATIO_BITS) / 4)
    {
      verFilter = 7;
    }
    else if (scalingRatio.second > (20 << SCALE_RATIO_BITS) / 7)
    {
      verFilter = 6;
    }
    else if (scalingRatio.second > (5 << SCALE_RATIO_BITS) / 2)
    {
      verFilter = 5;
    }
    else if (scalingRatio.second > (2 << SCALE_RATIO_BITS))
    {
      verFilter = 4;
    }
    else if (scalingRatio.second > (5 << SCALE_RATIO_BITS) / 3)
    {
      verFilter = 3;
    }
    else if (scalingRatio.second > (5 << SCALE_RATIO_BITS) / 4)
    {
      verFilter = 2;
    }
    else if (scalingRatio.second > (20 << SCALE_RATIO_BITS) / 19)
    {
      verFilter = 1;
    }

    filterHor = &DownsamplingFilterSRC[horFilter][0][0];
    filterVer = &DownsamplingFilterSRC[verFilter][0][0];
  }

  const int filterLength = downsampling ? 12 : (useLumaFilter ? NTAPS_LUMA : NTAPS_CHROMA);
  const int log2Norm = downsampling ? 14 : 12;

  int* buf = new int[orgHeight * scaledWidth];
  int maxVal = (1 << bitDepth) - 1;

  CHECK(bitDepth > 17, "Overflow may happen!");

  for (int i = 0; i < scaledWidth; i++)
  {
    const Pel* org = orgSrc;
    int refPos = (((i << compScale.first) - afterScaleLeftOffset) * scalingRatio.first + addX) >> posShiftX;
    int integer = refPos >> numFracShift;
    int frac = refPos & numFracPositions;
    int* tmp = buf + i;

    for (int j = 0; j < orgHeight; j++)
    {
      int sum = 0;
      const TFilterCoeff* f = filterHor + frac * filterLength;

      for (int k = 0; k < filterLength; k++)
      {
        int xInt = std::min<int>(std::max(0, integer + k - filterLength / 2 + 1), orgWidth - 1);
        sum += f[k] * org[xInt]; // postpone horizontal filtering gain removal after vertical filtering
      }

      *tmp = sum;

      tmp += scaledWidth;
      org += orgStride;
    }
  }

  Pel* dst = scaledSrc;

  for (int j = 0; j < scaledHeight; j++)
  {
    int refPos = (((j << compScale.second) - afterScaleTopOffset) * scalingRatio.second + addY) >> posShiftY;
    int integer = refPos >> numFracShift;
    int frac = refPos & numFracPositions;

    for (int i = 0; i < scaledWidth; i++)
    {
      int sum = 0;
      int* tmp = buf + i;
      const TFilterCoeff* f = filterVer + frac * filterLength;

      for (int k = 0; k < filterLength; k++)
      {
        int yInt = std::min<int>(std::max(0, integer + k - filterLength / 2 + 1), orgHeight - 1);
        sum += f[k] * tmp[yInt * scaledWidth];
      }

      dst[i] = std::min<int>(std::max(0, (sum + (1 << (log2Norm - 1))) >> log2Norm), maxVal);
    }

    dst += scaledStride;
  }

  delete[] buf;
}

void Picture::rescalePicture(const std::pair<int, int> scalingRatio,
                             const CPelUnitBuf& beforeScaling, const Window& scalingWindowBefore,
                             const PelUnitBuf& afterScaling, const Window& scalingWindowAfter,
                             const ChromaFormat chromaFormatIDC, const BitDepths& bitDepths, const bool useLumaFilter, const bool downsampling,
                             const bool horCollocatedChromaFlag, const bool verCollocatedChromaFlag)
{
  for (int comp = 0; comp < getNumberValidComponents(chromaFormatIDC); comp++)
  {
    ComponentID compID = ComponentID(comp);
    const CPelBuf& beforeScale = beforeScaling.get(compID);
    const PelBuf& afterScale = afterScaling.get(compID);

    sampleRateConv(scalingRatio, std::pair<int, int>(getComponentScaleX(compID, chromaFormatIDC), getComponentScaleY(compID, chromaFormatIDC)),
                   beforeScale, scalingWindowBefore.winLeftOffset * SPS::getWinUnitX(chromaFormatIDC), scalingWindowBefore.winTopOffset * SPS::getWinUnitY(chromaFormatIDC),
                   afterScale, scalingWindowAfter.winLeftOffset * SPS::getWinUnitX(chromaFormatIDC), scalingWindowAfter.winTopOffset * SPS::getWinUnitY(chromaFormatIDC),
                   bitDepths.recon[toChannelType(compID)], downsampling || useLumaFilter ? true : isLuma(compID), downsampling,
                   isLuma(compID) ? 1 : horCollocatedChromaFlag, isLuma(compID) ? 1 : verCollocatedChromaFlag);
  }
}
#endif

} // namespace vvenc

//! \}

