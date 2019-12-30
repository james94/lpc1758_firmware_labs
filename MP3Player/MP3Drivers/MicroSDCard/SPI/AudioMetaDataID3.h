/*
 * AudioMetaDataID3.h
 *
 *  Created on: May 12, 2018
 *      Author: james
 */

#ifndef AUDIOMETADATAID3_H_
#define AUDIOMETADATAID3_H_

#include "printf_lib.h"

/**
 * AudioMetaDataID is a class for reading meta-data
 * of ID3v2 for MP3 Files
 */

class AudioMetaDataID3 {
public:
    AudioMetaDataID3();
    virtual ~AudioMetaDataID3();
};

#endif /* AUDIOMETADATAID3_H_ */
