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


/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include <chrono>
#include <ctime>

#include "../vvencFFapp/EncApp.h"
#include "apputils/ParseArg.h"

#include "vvenc/vvenc.h"

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Main function
// ====================================================================================================================


int main(int argc, char* argv[])
{
  vvenc_set_logging_callback( nullptr, msgFnc ); // register global log callback ( deprecated, will be removed)

  std::string simdOpt;
  apputils::df::program_options_lite::Options opts;
  opts.addOptions()
    ( "c",           apputils::df::program_options_lite::parseConfigFile, "" )
    ( "SIMD",        simdOpt,         "" );

  apputils::df::program_options_lite::SilentReporter err;
  apputils::df::program_options_lite::scanArgv( opts, argc, ( const char** ) argv, err );

  vvenc_set_SIMD_extension( simdOpt.c_str() );

#if ENABLE_SPATIAL_SCALABLE
  std::fstream bitstream;
  vvencEncLibCommon* encLibCommon = vvenc_enclibcommon_create();

  std::vector<EncApp*> pcEncApp(1);
  bool resized = false;
  int layerIdx = 0;
  char** layerArgv = new char* [argc];

  do
  {
    pcEncApp[layerIdx] = new EncApp(bitstream, encLibCommon);

    // parse configuration per layer
    int j = 0;
    for (int i = 0; i < argc; i++)
    {
      if (argv[i][0] == '-' && argv[i][1] == 'l')
      {
        if (argc <= i + 1)
        {
          //THROW("Command line parsing error: missing parameter after -lx\n");
          std::cerr << "Command line parsing error: missing parameter after -lx\n" << std::endl;
          return 1;
        }
        int numParams = 1; // count how many parameters are consumed
        // check for long parameters, which start with "--"
        const std::string param = argv[i + 1];
        if (param.rfind("--", 0) != 0)
        {
          // only short parameters have a second parameter for the value
          if (argc <= i + 2)
          {
            //THROW("Command line parsing error: missing parameter after -lx\n");
            std::cerr << "Command line parsing error: missing parameter after -lx\n" << std::endl;
            return 1;
          }
          numParams++;
        }
        // check if correct layer index
        if (argv[i][2] == std::to_string(layerIdx).c_str()[0])
        {
          layerArgv[j] = argv[i + 1];
          if (numParams > 1)
          {
            layerArgv[j + 1] = argv[i + 2];
          }
          j += numParams;
        }
        i += numParams;
      }
      else
      {
        layerArgv[j] = argv[i];
        j++;
      }
    }

    if (!pcEncApp[layerIdx]->parseCfg(j, layerArgv))
    {
      pcEncApp[layerIdx]->destroy();
      return 1;
    }

    if (pcEncApp[layerIdx]->isShowVersionHelp())
    {
      return 0;
    }

    pcEncApp[layerIdx]->createLib(layerIdx);

    if (!resized)
    {
      pcEncApp.resize(pcEncApp[layerIdx]->getMaxLayers());
      resized = true;
    }

    layerIdx++;
  } while (layerIdx < (int)pcEncApp.size());

  delete[] layerArgv;

  // Consistency check between layers
  if (!pcEncApp[0]->isDecode() && layerIdx > 1) {
    for (int i = 0; i < layerIdx; i++) {
      if (pcEncApp[i]->check(pcEncApp) != VVENC_OK) {
        return 1;
      }
    }
  }
#else
  EncApp* pcEncApp = new EncApp;

  // parse configuration
  if ( ! pcEncApp->parseCfg( argc, argv ) )
  {
    return 1;
  }

  if( pcEncApp->isShowVersionHelp() )
  {
    return 0;
  }
#endif

  // starting time
  auto startTime  = std::chrono::steady_clock::now();
  std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  msgApp( VVENC_INFO, " started @ %s", std::ctime(&startTime2) );
  clock_t startClock = clock();

#if ENABLE_SPATIAL_SCALABLE
  int ret = VVENC_OK;

  if (pcEncApp[0]->isDecode()) {
      ret = pcEncApp[0]->decode();
  }
  else {
    for (int pass = 0; pass < pcEncApp[0]->getRCNumPasses(); pass++) {
      // initialize encoder pass
      for (auto& encApp : pcEncApp) {
        if (encApp->initPass(pass) != VVENC_OK) {
          return 1;
        }
      }
      // main loop
      bool inputDone = false;
      while (!inputDone) {
        bool encDone = false;
        while (!encDone) {
          for (auto& encApp : pcEncApp) {
            ret = encApp->encode(inputDone, encDone);
            if (ret != VVENC_OK) {
              return 1;
            }
          }
        }
      }
      for (auto& encApp : pcEncApp) {
        encApp->closeYuvInputFile();
      }
      // reset encCommonLib before next pass
      if (pass + 1 < pcEncApp[0]->getRCNumPasses()) {
        vvenc_enclibcommon_reset(encLibCommon);
      }
    }
  }
#else
  // call encoding function
  int ret = pcEncApp->encode();
#endif

  // ending time
  clock_t endClock = clock();
  auto endTime = std::chrono::steady_clock::now();
  std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  auto encTime = std::chrono::duration_cast<std::chrono::milliseconds>( endTime - startTime).count();

#if ENABLE_SPATIAL_SCALABLE
  for (auto& encApp : pcEncApp) {
    encApp->destroy();
    delete encApp;
  }
  vvenc_enclibcommon_close(encLibCommon);
#else
  delete pcEncApp;
#endif

  msgApp( VVENC_INFO, "\n finished @ %s", std::ctime(&endTime2) );
  msgApp( VVENC_INFO, " Total Time: %12.3f sec. [user] %12.3f sec. [elapsed]\n", (endClock - startClock) * 1.0 / CLOCKS_PER_SEC, encTime / 1000.0);

  return ret;
}

//! \}
