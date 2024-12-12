#ifndef ChildNode_HEADER_GUARD_
#define ChildNode_HEADER_GUARD_

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
 * $Id: ChildNode.hpp 568078 2007-08-21 11:43:25Z amassari $
 */

//
//  This file is part of the internal implementation of the C++ XML DOM.
//  It should NOT be included or used directly by application programs.
//
//  Applications should include the file <xercesc/dom/deprecated/DOM.hpp> for the entire
//  DOM API, or DOM_*.hpp for individual DOM classes, where the class
//  name is substituded for the *.
//

/**
 * ChildNode adds to NodeImpl the capability of being a child, this is having
 * siblings.
 **/

#include "NodeImpl.hpp"

XERCES_CPP_NAMESPACE_BEGIN


class DEPRECATED_DOM_EXPORT ChildNode: public NodeImpl {
public:
    ChildNode                *previousSibling;
    ChildNode                *nextSibling;

public:
    ChildNode(DocumentImpl *ownerDocument);
    ChildNode(const ChildNode &other);
    virtual ~ChildNode();

    virtual NodeImpl * getNextSibling();
    virtual NodeImpl * getParentNode();
    virtual NodeImpl*  getPreviousSibling();
};


XERCES_CPP_NAMESPACE_END

#endif
