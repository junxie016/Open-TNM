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
 * $Id: BitSet.hpp 568078 2007-08-21 11:43:25Z amassari $
 */

#if !defined(BITSET_HPP)
#define BITSET_HPP

#include <xercesc/util/XMemory.hpp>
#include <xercesc/util/PlatformUtils.hpp>

XERCES_CPP_NAMESPACE_BEGIN

class XMLUTIL_EXPORT BitSet : public XMemory
{
public:
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    BitSet( const unsigned int size
          , MemoryManager* const manager = XMLPlatformUtils::fgMemoryManager);
    BitSet(const BitSet& toCopy);
    ~BitSet();


    // -----------------------------------------------------------------------
    //  Equality methods
    // -----------------------------------------------------------------------
    bool equals(const BitSet& other) const;


    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------
    bool allAreCleared() const;
    bool allAreSet() const;
    unsigned int size() const;
    bool get(const unsigned int index) const;


    // -----------------------------------------------------------------------
    //  Setter methods
    // -----------------------------------------------------------------------
    void clear(const unsigned int index);
    void clearAll();
    void set(const unsigned int index);


    // -----------------------------------------------------------------------
    //  Bitwise logical operations
    // -----------------------------------------------------------------------
    void andWith(const BitSet& other);
    void orWith(const BitSet& other);
    void xorWith(const BitSet& other);


    // -----------------------------------------------------------------------
    //  Miscellaneous
    // -----------------------------------------------------------------------
    unsigned int hash(const unsigned int hashModulus) const;


private :
    // -----------------------------------------------------------------------
    //  Unimplemented constructors
    // -----------------------------------------------------------------------
    BitSet();    
    BitSet& operator=(const BitSet&);
    // -----------------------------------------------------------------------
    //  Private methods
    // -----------------------------------------------------------------------
    void ensureCapacity(const unsigned int bits);


    // -----------------------------------------------------------------------
    //  Data members
    //
    //  fBits
    //      The array of unsigned longs used to store the bits.
    //
    //  fUnitLen
    //      The length of the storage array, in storage units not bits.
    // -----------------------------------------------------------------------
    MemoryManager*  fMemoryManager;
    unsigned long*  fBits;
    unsigned int    fUnitLen;
};

XERCES_CPP_NAMESPACE_END

#endif
