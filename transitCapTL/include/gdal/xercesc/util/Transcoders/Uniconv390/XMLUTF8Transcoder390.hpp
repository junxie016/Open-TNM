/*
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * $Id: XMLUTF8Transcoder390.hpp 568078 2007-08-21 11:43:25Z amassari $
 */

#ifndef XMLUTF8TRANSCODER390_HPP
#define XMLUTF8TRANSCODER390_HPP

#include <xercesc/util/XercesDefs.hpp>
#include <xercesc/util/TransService.hpp>
#include <xercesc/util/UTFDataFormatException.hpp>

XERCES_CPP_NAMESPACE_BEGIN

//
//  This class provides an implementation of the XMLTranscoder interface
//  for a simple UTF8 transcoder. The parser does some encodings
//  intrinsically without depending upon external transcoding services.
//  To make everything more orthagonal, we implement these internal
//  transcoders using the same transcoder abstraction as the pluggable
//  transcoding services do.
//
class XMLUTIL_EXPORT XMLUTF8Transcoder390 : public XMLTranscoder
{
public :
    // -----------------------------------------------------------------------
    //  Public constructors and destructor
    // -----------------------------------------------------------------------
    XMLUTF8Transcoder390
    (
        const   XMLCh* const    encodingName
        , const unsigned int    blockSize
        , MemoryManager* const  manager = XMLPlatformUtils::fgMemoryManager
    );

    virtual ~XMLUTF8Transcoder390();


    // -----------------------------------------------------------------------
    //  Implementation of the XMLTranscoder interface
    // -----------------------------------------------------------------------
    virtual unsigned int transcodeFrom
    (
        const   XMLByte* const          srcData
        , const unsigned int            srcCount
        ,       XMLCh* const            toFill
        , const unsigned int            maxChars
        ,       unsigned int&           bytesEaten
        ,       unsigned char* const    charSizes
    );

    virtual unsigned int transcodeTo
    (
        const   XMLCh* const    srcData
        , const unsigned int    srcCount
        ,       XMLByte* const  toFill
        , const unsigned int    maxBytes
        ,       unsigned int&   charsEaten
        , const UnRepOpts       options
    );

    virtual bool canTranscodeTo
    (
        const   unsigned int    toCheck
    )   const;


private :

    inline void checkTrailingBytes(
                                    const XMLByte      toCheck
                                  , const unsigned int trailingBytes
                                  , const unsigned int position       
                                  ) const;

private :
    // -----------------------------------------------------------------------
    //  Unimplemented constructors and operators
    // -----------------------------------------------------------------------
    XMLUTF8Transcoder390(const XMLUTF8Transcoder390&);
    XMLUTF8Transcoder390& operator=(const XMLUTF8Transcoder390&);
};

inline 
void XMLUTF8Transcoder390::checkTrailingBytes(const XMLByte      toCheck
                                            , const unsigned int trailingBytes
                                            , const unsigned int position) const
{

    if((toCheck & 0xC0) != 0x80) 
    {
        char len[2]  = {(char)(trailingBytes+0x31), 0};
        char pos[2]  = {(char)(position+0x31), 0};
        char byte[2] = {toCheck,0};
        ThrowXMLwithMemMgr3(UTFDataFormatException, XMLExcepts::UTF8_FormatError, pos, byte, len, getMemoryManager());
    }

}

XERCES_CPP_NAMESPACE_END

#endif
