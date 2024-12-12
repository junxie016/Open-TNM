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
 * $Id: XercesGroupInfo.hpp 568078 2007-08-21 11:43:25Z amassari $
 */

#if !defined(XERCESGROUPINFO_HPP)
#define XERCESGROUPINFO_HPP


/**
  * The class act as a place holder to store group information.
  *
  * The class is intended for internal use.
  */

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------
#include <xercesc/util/RefVectorOf.hpp>
#include <xercesc/validators/schema/SchemaElementDecl.hpp>

#include <xercesc/internal/XSerializable.hpp>

XERCES_CPP_NAMESPACE_BEGIN

// ---------------------------------------------------------------------------
//  Forward Declarations
// ---------------------------------------------------------------------------
class ContentSpecNode;
class XSDLocator;


class VALIDATORS_EXPORT XercesGroupInfo : public XSerializable, public XMemory
{
public:
    // -----------------------------------------------------------------------
    //  Public Constructors/Destructor
    // -----------------------------------------------------------------------
    XercesGroupInfo
    (
        unsigned int groupNameId
        , unsigned int groupNamespaceId
        , MemoryManager* const manager = XMLPlatformUtils::fgMemoryManager
    );
    ~XercesGroupInfo();

    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------
    bool                     getCheckElementConsistency() const;
    int                      getScope() const;
    unsigned int             elementCount() const;
    ContentSpecNode*         getContentSpec() const;
    SchemaElementDecl*       elementAt(const unsigned int index);
    const SchemaElementDecl* elementAt(const unsigned int index) const;
    XSDLocator*              getLocator() const;
    XercesGroupInfo*         getBaseGroup() const;
    unsigned int             getNameId() const;
    unsigned int             getNamespaceId() const;

	// -----------------------------------------------------------------------
    //  Setter methods
    // -----------------------------------------------------------------------
    void setScope(const int other);
    void setContentSpec(ContentSpecNode* const other);
    void addElement(SchemaElementDecl* const toAdd);
    void setLocator(XSDLocator* const aLocator);
    void setBaseGroup(XercesGroupInfo* const baseGroup);
    void setCheckElementConsistency(const bool aValue);

    /***
     * Support for Serialization/De-serialization
     ***/
    DECL_XSERIALIZABLE(XercesGroupInfo)
    XercesGroupInfo(MemoryManager* const manager = XMLPlatformUtils::fgMemoryManager);

private:
    // -----------------------------------------------------------------------
    //  Unimplemented contstructors and operators
    // -----------------------------------------------------------------------
    XercesGroupInfo(const XercesGroupInfo& elemInfo);
    XercesGroupInfo& operator= (const XercesGroupInfo& other);

    // -----------------------------------------------------------------------
    //  Private data members
    // -----------------------------------------------------------------------
    bool                            fCheckElementConsistency;
    int                             fScope;
    unsigned int                    fNameId;
    unsigned int                    fNamespaceId;
    ContentSpecNode*                fContentSpec;
    RefVectorOf<SchemaElementDecl>* fElements;
    XercesGroupInfo*                fBaseGroup; // redefine by restriction
    XSDLocator*                     fLocator;
};

// ---------------------------------------------------------------------------
//  XercesGroupInfo: Getter methods
// ---------------------------------------------------------------------------
inline int XercesGroupInfo::getScope() const {

    return fScope;
}

inline unsigned int XercesGroupInfo::elementCount() const {

    return fElements->size();
}

inline ContentSpecNode* XercesGroupInfo::getContentSpec() const {

    return fContentSpec;
}

inline SchemaElementDecl*
XercesGroupInfo::elementAt(const unsigned int index) {

    return fElements->elementAt(index);
}

inline const SchemaElementDecl*
XercesGroupInfo::elementAt(const unsigned int index) const {

    return fElements->elementAt(index);
}

inline XSDLocator* XercesGroupInfo::getLocator() const {

    return fLocator;
}

inline XercesGroupInfo* XercesGroupInfo::getBaseGroup() const {

    return fBaseGroup;
}

inline bool XercesGroupInfo::getCheckElementConsistency() const {

    return fCheckElementConsistency;
}

inline unsigned int XercesGroupInfo::getNameId() const
{
    return fNameId;
}

inline unsigned int XercesGroupInfo::getNamespaceId() const
{
    return fNamespaceId;
}

// ---------------------------------------------------------------------------
//  XercesGroupInfo: Setter methods
// ---------------------------------------------------------------------------}
inline void XercesGroupInfo::setScope(const int other) {

    fScope = other;
}

inline void XercesGroupInfo::setContentSpec(ContentSpecNode* const other) {

    fContentSpec = other;
}

inline void XercesGroupInfo::addElement(SchemaElementDecl* const elem) {

    if (!fElements->containsElement(elem))
        fElements->addElement(elem);
}

inline void XercesGroupInfo::setBaseGroup(XercesGroupInfo* const baseGroup) {

    fBaseGroup = baseGroup;
}

inline void XercesGroupInfo::setCheckElementConsistency(const bool aValue) {

    fCheckElementConsistency = aValue;
}

XERCES_CPP_NAMESPACE_END

#endif

/**
  * End of file XercesGroupInfo.hpp
  */

