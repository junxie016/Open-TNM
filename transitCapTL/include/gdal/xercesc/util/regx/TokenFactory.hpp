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
 * $Id: TokenFactory.hpp 568078 2007-08-21 11:43:25Z amassari $
 */

#if !defined(TOKENFACTORY_HPP)
#define TOKENFACTORY_HPP

// ---------------------------------------------------------------------------
//  Includes
// ---------------------------------------------------------------------------
#include <xercesc/util/RefVectorOf.hpp>
#include <xercesc/util/regx/Token.hpp>
#include <xercesc/util/Mutexes.hpp>

XERCES_CPP_NAMESPACE_BEGIN

// ---------------------------------------------------------------------------
//  Forward Declaration
// ---------------------------------------------------------------------------
class RangeToken;
class CharToken;
class ClosureToken;
class ConditionToken;
class ConcatToken;
class ModifierToken;
class ParenToken;
class StringToken;
class UnionToken;

class XMLUTIL_EXPORT TokenFactory : public XMemory
{

public:
	// -----------------------------------------------------------------------
    //  Constructors and destructors
    // -----------------------------------------------------------------------
    TokenFactory(MemoryManager* const manager = XMLPlatformUtils::fgMemoryManager);
    ~TokenFactory();

    // -----------------------------------------------------------------------
    //  Factory methods
    // -----------------------------------------------------------------------
    Token* createToken(const unsigned short tokType);

    ParenToken* createLook(const unsigned short tokType, Token* const token);
    ParenToken* createParenthesis(Token* const token, const int noGroups);
    ClosureToken* createClosure(Token* const token, bool isNonGreedy = false);
    ConcatToken* createConcat(Token* const token1, Token* const token2);
    UnionToken* createUnion(const bool isConcat = false);
    RangeToken* createRange(const bool isNegRange = false);
    CharToken* createChar(const XMLUInt32 ch, const bool isAnchor = false);
    StringToken* createBackReference(const int refNo);
    StringToken* createString(const XMLCh* const literal);
    ModifierToken* createModifierGroup(Token* const child,
                                       const int add, const int mask);
    ConditionToken* createCondition(const int refNo, Token* const condition,
                                    Token* const yesFlow, Token* const noFlow);


	//static void printUnicode();

    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------
    /*
     *  Gets a commonly used RangeToken from the token registry based on the
     *  range name.
     */
    RangeToken* getRange(const XMLCh* const name,const bool complement=false);
    Token* getLineBegin();
	Token* getLineBegin2();
    Token* getLineEnd();
    Token* getStringBegin();
    Token* getStringEnd();
    Token* getStringEnd2();
    Token* getWordEdge();
    Token* getNotWordEdge();
    Token* getWordBegin();
    Token* getWordEnd();
    Token* getDot();
	Token* getCombiningCharacterSequence();
	Token* getGraphemePattern();
    MemoryManager* getMemoryManager() const;

    static RangeToken* staticGetRange(const XMLCh* const name,const bool complement=false);

    // -----------------------------------------------------------------------
    //  Notification that lazy data has been deleted
    // -----------------------------------------------------------------------
	static void reinitTokenFactoryMutex();

private:
    // -----------------------------------------------------------------------
    //  Unimplemented constructors and operators
    // -----------------------------------------------------------------------
    TokenFactory(const TokenFactory&);
    TokenFactory& operator=(const TokenFactory&);

    // -----------------------------------------------------------------------
    //  Private data members
    //
    //  fRangeInitialized
    //      Indicates whether we have initialized the RangeFactory instance or
    //      not
	//		
    //  fToken
    //      Contains user created Token objects. Used for memory cleanup.
    // -----------------------------------------------------------------------
    RefVectorOf<Token>* fTokens;
    Token*              fEmpty;
    Token*              fLineBegin;
    Token*              fLineBegin2;
    Token*              fLineEnd;
    Token*              fStringBegin;
    Token*              fStringEnd;
    Token*              fStringEnd2;
    Token*              fWordEdge;
    Token*              fNotWordEdge;
    Token*              fWordEnd;
    Token*              fWordBegin;
    Token*              fDot;
    Token*              fCombiningChar;
    Token*              fGrapheme;
    MemoryManager*      fMemoryManager;
};

inline RangeToken* TokenFactory::getRange(const XMLCh* const name,const bool complement)
{
    return staticGetRange(name, complement);
}

inline MemoryManager* TokenFactory::getMemoryManager() const
{
    return fMemoryManager;
}

XERCES_CPP_NAMESPACE_END

#endif

/**
  *	End file TokenFactory
  */

