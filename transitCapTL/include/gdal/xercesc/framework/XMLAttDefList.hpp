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
 * $Id: XMLAttDefList.hpp 568078 2007-08-21 11:43:25Z amassari $
 */

#if !defined(XMLATTDEFLIST_HPP)
#define XMLATTDEFLIST_HPP

#include <xercesc/util/XercesDefs.hpp>
#include <xercesc/util/XMemory.hpp>
#include <xercesc/internal/XSerializable.hpp>

XERCES_CPP_NAMESPACE_BEGIN

class XMLAttDef;

/**
 *  This class defines an abstract interface that all validators must support.
 *  When the scanner scans the attributes in a start tag, it must have a list
 *  of the defined attributes for that element. This is used to fault in
 *  defaulted and fixed attributes, to know which ones are required, and to
 *  know the their types in order to do the correct normalization.
 *
 *  Since each validator will have its own derivatives of XMLAttDef and will
 *  have its own specialized storage mechanisms for elements and the att
 *  defs that they own, there must be an abstracted way for the scanner to
 *  deal with this list.
 *
 *  It does not derive from the generic Enumerator template class, because
 *  there are portability issues with deriving from a template class in a
 *  DLL. It does though provide a similar enumerator interface.
 */

class XMLPARSER_EXPORT XMLAttDefList : public XSerializable, public XMemory
{
public:
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------

    /** @name Destructor */
    //@{
    virtual ~XMLAttDefList();
    //@}


    // -----------------------------------------------------------------------
    //  The virtual interface
    // -----------------------------------------------------------------------

    /** 
     * @deprecated This method is not thread-safe.
     */
    virtual bool hasMoreElements() const = 0;
    virtual bool isEmpty() const = 0;
    virtual XMLAttDef* findAttDef
    (
        const   unsigned long       uriID
        , const XMLCh* const        attName
    ) = 0;
    virtual const XMLAttDef* findAttDef
    (
        const   unsigned long       uriID
        , const XMLCh* const        attName
    )   const = 0;
    virtual XMLAttDef* findAttDef
    (
        const   XMLCh* const        attURI
        , const XMLCh* const        attName
    ) = 0;
    virtual const XMLAttDef* findAttDef
    (
        const   XMLCh* const        attURI
        , const XMLCh* const        attName
    )   const = 0;

    /** 
     * @deprecated This method is not thread-safe.
     */
    virtual XMLAttDef& nextElement() = 0;

    /** 
     * @deprecated This method is not thread-safe.
     */
    virtual void Reset() = 0;

    /**
     * return total number of attributes in this list
     */
    virtual unsigned int getAttDefCount() const = 0;

    /**
     * return attribute at the index-th position in the list.
     */
    virtual XMLAttDef &getAttDef(unsigned int index) = 0;

    /**
     * return attribute at the index-th position in the list.
     */
    virtual const XMLAttDef &getAttDef(unsigned int index) const = 0;

    /***
     * Support for Serialization/De-serialization
     ***/
    DECL_XSERIALIZABLE(XMLAttDefList)


    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------

    /** @name Getter methods */
    //@{

    /** Get the memory manager
      *
      * This method returns the configurable memory manager used by the
      * element declaration for dynamic allocation/deacllocation.
      *
      * @return the memory manager
      */
    MemoryManager* getMemoryManager() const;

    //@}

protected :
    // -----------------------------------------------------------------------
    //  Hidden constructors and operators
    // -----------------------------------------------------------------------
    XMLAttDefList(MemoryManager* const manager = XMLPlatformUtils::fgMemoryManager);

private:
    // unimplemented
    XMLAttDefList(const XMLAttDefList&);
    XMLAttDefList& operator=(const XMLAttDefList&);

    MemoryManager*      fMemoryManager;
};



// ---------------------------------------------------------------------------
//  XMLAttDefList: Getter methods
// ---------------------------------------------------------------------------

inline MemoryManager* XMLAttDefList::getMemoryManager() const
{
    return fMemoryManager;
}

// ---------------------------------------------------------------------------
//  XMLAttDefList: Constructors and Destructor
// ---------------------------------------------------------------------------
inline XMLAttDefList::~XMLAttDefList()
{
}


// ---------------------------------------------------------------------------
//  XMLAttDefList: Protected Constructor
// ---------------------------------------------------------------------------
inline XMLAttDefList::XMLAttDefList(MemoryManager* const manager):
fMemoryManager(manager)
{
}

XERCES_CPP_NAMESPACE_END

#endif
