/*========================================================================================
  A general purpose predicator function for STL. It is treated as part of TNM, but actually
  it has nothing to do with TNM. It can be used anywhere.
  This code is based on a program I found from Website, but I forgot to record Author's 
  name.

  This following shows an example how this can be used.
class Person 
{
private:
	string	_name;
	int	_salary;
	int	_id;
public:
	string	name()		{ return _name; }
	int	salary()	{ return _salary; }
	int	id()		{ return _id; }
};

void func()
{
	vector<Person> v;

	// Add a lot of persons to the vector...

	vector<Person>::iterator i, a(v.begin()), b(v.end());

	i = find_if(a, b, pred(&Person::name, "Niklas"));
	i = find_if(a, b, pred(&Person::id, 17));
	i = find_if(a, b, pred(&Person::salary, 3500, greater<int>()));
}
///////////////////////////////////////////////////////////////////////////////////////////
    if container contais objects's pointer instead of object themsevles, use
	predP function instead of pred. the usage are same.

=========================================================================================*/
#ifndef MY_PREDICAT_H
#define MY_PREDICAT_H

#include <functional>
#include <iostream>
using namespace std; 
/***********************************************************
 T: the object type stored in the constainer.
 R: the atrribute of the object to be compared
 V: still the attribute of the object to be compared.
    == operator should be defined for class V.
	*********************************************************/
template <class T, class R, class V, class Comparator>
class PredSelector : public unary_function<T,bool> //a derived class from unaray_function class
{
protected:
   R           (T::*_f)();
   V           _v;
   Comparator  _comp;
public:
   PredSelector(R (T::*f)(), const V &v, Comparator comp)
      : _f(f), _v(v), _comp(comp)      {}

   // "t" can't be "const" because "T::_f()" isn't "const":
    bool operator()(T &t) const         { return _comp((t.*_f)(), _v); }

};


/*  function pred: to find an object in constainer whose key is equivlaent to
    the specified value v*/
template <class T, class R, class V>
inline PredSelector<T,R,V,equal_to<V> > pred(R (T::*f)(), const V &v)
{
   return PredSelector<T,R,V,equal_to<V> >(f, v, equal_to<V>());
}

/* another version of function pred: to find objects in containter whose keys satisfied
   a relationship between the specifie value v. the relationship can be any functionals
   defined in stl, e.g. greater<double>*/
template <class T, class R, class V, class Comp>
inline PredSelector<T,R,V,Comp> predC(R (T::*f)(), const V &v, Comp comp)
{
   return PredSelector<T,R,V,Comp>(f, v, comp);
}


//***********************************************************

template <class T, class R, class V, class Comparator>
class PredCSelector : public unary_function<T,bool>
{
protected:
   R           (T::*_f)() const;
   V           _v;
   Comparator  _comp;
public:
   PredCSelector(R (T::*f)() const, const V &v, Comparator comp)
      : _f(f), _v(v), _comp(comp)      {}
   bool operator()(const T &t) const   { return _comp((t.*_f)(), _v); }
  //this is used when vector stores object.
//   bool operator()(const T *t) const   { return _comp((t->*_f)(), _v); } 
// this is used when vector stores object pointer, instead of object itself.

};


template <class T, class R, class V>
inline PredCSelector<T,R,V,equal_to<V> > pred(R (T::*f)() const, const V &v)
{
   return PredCSelector<T,R,V,equal_to<V> >(f, v, equal_to<V>());
}


template <class T, class R, class V, class Comp>
inline PredCSelector<T,R,V,Comp> predC(R (T::*f)() const, const V &v, Comp comp)
{
   return PredCSelector<T,R,V,Comp>(f, v, comp);
}

/*=======================================================================================
 if containers contains objects' pointers.
 =======================================================================================*/
template <class T, class R, class V, class Comparator>
class PredSelectorP : public unary_function<T,bool> //a derived class from unaray_function class
{
protected:
   R           (T::*_f)();
   V           _v;
   Comparator  _comp;
public:
   PredSelectorP(R (T::*f)(), const V &v, Comparator comp)
      : _f(f), _v(v), _comp(comp)      {}

   // "t" can't be "const" because "T::_f()" isn't "const":
     bool operator()(T *t) const         { return _comp((t->*_f)(), _v); }

};


/*  function predP:  v*/
template <class T, class R, class V>
inline PredSelectorP<T,R,V,equal_to<V> > predP(R (T::*f)(), const V &v)
{
   return PredSelectorP<T,R,V,equal_to<V> >(f, v, equal_to<V>());
}

/* another version of function predP: */
template <class T, class R, class V, class Comp>
inline PredSelectorP<T,R,V,Comp> predPC(R (T::*f)(), const V &v, Comp comp)
{
   return PredSelectorP<T,R,V,Comp>(f, v, comp);
}


//***********************************************************

template <class T, class R, class V, class Comparator>
class PredCSelectorP : public unary_function<T,bool>
{
protected:
   R           (T::*_f)() const;
   V           _v;
   Comparator  _comp;
public:
   PredCSelectorP(R (T::*f)() const, const V &v, Comparator comp)
      : _f(f), _v(v), _comp(comp)      {}
  //this is used when vector stores object.
   bool operator()(const T *t) const   { return _comp((t->*_f)(), _v); } 
// this is used when vector stores object pointer, instead of object itself.

};


template <class T, class R, class V>
inline PredCSelectorP<T,R,V,equal_to<V> > predP(R (T::*f)() const, const V &v)
{
   return PredCSelectorP<T,R,V,equal_to<V> >(f, v, equal_to<V>());
}


template <class T, class R, class V, class Comp>
inline PredCSelectorP<T,R,V,Comp> predPC(R (T::*f)() const, const V &v, Comp comp)
{
   return PredCSelectorP<T,R,V,Comp>(f, v, comp);
}
//template <class T>


//***********************************************************

#endif
