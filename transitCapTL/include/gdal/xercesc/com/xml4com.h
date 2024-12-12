

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 7.00.0555 */
/* at Sat Apr 24 22:57:13 2010
 */
/* Compiler settings for ..\..\..\..\..\src\xercesc\com\xml4com.idl:
    Oicf, W1, Zp8, env=Win64 (32b run), target_arch=AMD64 7.00.0555 
    protocol : dce , ms_ext, c_ext, robust
    error checks: allocation ref bounds_check enum stub_data 
    VC __declspec() decoration level: 
         __declspec(uuid()), __declspec(selectany), __declspec(novtable)
         DECLSPEC_UUID(), MIDL_INTERFACE()
*/
/* @@MIDL_FILE_HEADING(  ) */

#pragma warning( disable: 4049 )  /* more than 64k source lines */


/* verify that the <rpcndr.h> version is high enough to compile this file*/
#ifndef __REQUIRED_RPCNDR_H_VERSION__
#define __REQUIRED_RPCNDR_H_VERSION__ 475
#endif

#include "rpc.h"
#include "rpcndr.h"

#ifndef __RPCNDR_H_VERSION__
#error this stub requires an updated version of <rpcndr.h>
#endif // __RPCNDR_H_VERSION__


#ifndef __xml4com_h__
#define __xml4com_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __IIBMXMLDOMNodeIdentity_FWD_DEFINED__
#define __IIBMXMLDOMNodeIdentity_FWD_DEFINED__
typedef interface IIBMXMLDOMNodeIdentity IIBMXMLDOMNodeIdentity;
#endif 	/* __IIBMXMLDOMNodeIdentity_FWD_DEFINED__ */


#ifndef __DOMDocument_FWD_DEFINED__
#define __DOMDocument_FWD_DEFINED__

#ifdef __cplusplus
typedef class DOMDocument DOMDocument;
#else
typedef struct DOMDocument DOMDocument;
#endif /* __cplusplus */

#endif 	/* __DOMDocument_FWD_DEFINED__ */


#ifndef __XMLHTTPRequest_FWD_DEFINED__
#define __XMLHTTPRequest_FWD_DEFINED__

#ifdef __cplusplus
typedef class XMLHTTPRequest XMLHTTPRequest;
#else
typedef struct XMLHTTPRequest XMLHTTPRequest;
#endif /* __cplusplus */

#endif 	/* __XMLHTTPRequest_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"

#ifdef __cplusplus
extern "C"{
#endif 



#ifndef __Xerces_LIBRARY_DEFINED__
#define __Xerces_LIBRARY_DEFINED__

/* library Xerces */
/* [helpstring][version][uuid] */ 


















EXTERN_C const IID LIBID_Xerces;

#ifndef __IIBMXMLDOMNodeIdentity_INTERFACE_DEFINED__
#define __IIBMXMLDOMNodeIdentity_INTERFACE_DEFINED__

/* interface IIBMXMLDOMNodeIdentity */
/* [unique][helpstring][oleautomation][hidden][uuid][object] */ 


EXTERN_C const IID IID_IIBMXMLDOMNodeIdentity;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("35ADBB42-47B2-4b22-9D2E-1DA260EE5007")
    IIBMXMLDOMNodeIdentity : public IUnknown
    {
    public:
        virtual /* [helpstring][propget] */ HRESULT STDMETHODCALLTYPE get_NodeId( 
            /* [retval][out] */ long *pVal) = 0;
        
    };
    
#else 	/* C style interface */

    typedef struct IIBMXMLDOMNodeIdentityVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IIBMXMLDOMNodeIdentity * This,
            /* [in] */ REFIID riid,
            /* [iid_is][out] */ void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IIBMXMLDOMNodeIdentity * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IIBMXMLDOMNodeIdentity * This);
        
        /* [helpstring][propget] */ HRESULT ( STDMETHODCALLTYPE *get_NodeId )( 
            IIBMXMLDOMNodeIdentity * This,
            /* [retval][out] */ long *pVal);
        
        END_INTERFACE
    } IIBMXMLDOMNodeIdentityVtbl;

    interface IIBMXMLDOMNodeIdentity
    {
        CONST_VTBL struct IIBMXMLDOMNodeIdentityVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IIBMXMLDOMNodeIdentity_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IIBMXMLDOMNodeIdentity_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IIBMXMLDOMNodeIdentity_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IIBMXMLDOMNodeIdentity_get_NodeId(This,pVal)	\
    ( (This)->lpVtbl -> get_NodeId(This,pVal) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IIBMXMLDOMNodeIdentity_INTERFACE_DEFINED__ */


EXTERN_C const CLSID CLSID_DOMDocument;

#ifdef __cplusplus

class DECLSPEC_UUID("C4775B61-ECD6-11d4-94B4-00A0CC57CBCB")
DOMDocument;
#endif

EXTERN_C const CLSID CLSID_XMLHTTPRequest;

#ifdef __cplusplus

class DECLSPEC_UUID("3A725053-15FB-4065-9171-BC02BCF3876C")
XMLHTTPRequest;
#endif
#endif /* __Xerces_LIBRARY_DEFINED__ */

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


