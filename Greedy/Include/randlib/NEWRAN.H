// newran.h ------------------------------------------------------------

// NEWRAN02B - 22 July 2002

#ifndef NEWRAN_LIB
#define NEWRAN_LIB 0

//******************* utilities and definitions *************************

#include "include.h"
#include "boolean.h"
#include "myexcept.h"
#include "extreal.h"

#ifdef use_namespace
namespace NEWRAN { using namespace RBD_COMMON; }
namespace RBD_LIBRARIES { using namespace NEWRAN; }
namespace NEWRAN {
#endif

typedef Real (*PDF)(Real);                // probability density function

Real ln_gamma(Real);                      // log gamma function

//**************** uniform random number generator **********************

class RepeatedRandom;
class SelectedRandom;

class RAND_EXT_CLASS MyRandom                              // uniform random number generator
{
   static double seed;                    // seed
   //static unsigned long iseed;          // for Mother
   static Real Buffer[128];               // for mixing random numbers
   static Real Raw();                     // unmixed random numbers
   void operator=(const MyRandom&) {}       // private so can't access

public:
   static void Set(double s);             // set seed (0 < seed < 1)
   static double Get();                   // get seed
   virtual Real Next();                   // get new value
   virtual char* Name();                  // identification
   virtual Real Density(Real) const;      // used by PosGen & Asymgen
   MyRandom() {}                            // do nothing
   virtual ~MyRandom() {}                   // make destructors virtual
   SelectedRandom& operator()(double);    // used by MixedRandom
   RepeatedRandom& operator()(int);       // for making combinations
   virtual ExtReal Mean() const { return 0.5; }
                                          // mean of distribution
   virtual ExtReal Variance() const { return 1.0/12.0; }
					  // variance of distribution
   virtual void tDelete() {}              // delete components of sum
   virtual int nelems() const { return 1; }
                                          // used by MixedRandom
   virtual void load(int*,Real*,MyRandom**);
   friend class RandomPermutation;
};


//****************** uniform random number generator *********************

class RAND_EXT_CLASS Uniform : public MyRandom
{
   void operator=(const Uniform&) {}      // private so can't access

public:
   char* Name();                          // identification
   Uniform() {}                           // set value
   Real Next() { return MyRandom::Next(); }
   ExtReal Mean() const { return 0.5; }
   ExtReal Variance() const { return 1.0/12.0; }
   Real Density(Real x) const { return (x < 0.0 || x > 1.0) ? 0 : 1.0; }
};


//************************* return constant ******************************

class RAND_EXT_CLASS Constant : public MyRandom
{
   void operator=(const Constant&) {}     // private so can't access
   Real value;                            // value to be returned

public:
   char* Name();                          // identification
   Constant(Real v) { value=v; }          // set value
//   Real Next();
   Real Next() { return value; }
   ExtReal Mean() const { return value; }
   ExtReal Variance() const { return 0.0; }
};

//**************** positive random number generator **********************

class RAND_EXT_CLASS PosGen : public MyRandom              // generate positive rv
{
   void operator=(const PosGen&) {}       // private so can't access

protected:
   Real xi, *sx, *sfx;
   bool NotReady;
   void Build(bool);                      // called on first call to Next

public:
   char* Name();                          // identification
   PosGen();                              // constructor
   ~PosGen();                             // destructor
   Real Next();                           // to get a single new value
   ExtReal Mean() const { return (ExtReal)Missing; }
   ExtReal Variance() const { return (ExtReal)Missing; }
};

//**************** symmetric random number generator **********************

class RAND_EXT_CLASS SymGen : public PosGen              // generate symmetric rv
{
   void operator=(const SymGen&) {}       // private so can't access

public:
   char* Name();                          // identification
   Real Next();                           // to get a single new value
};

//**************** normal random number generator **********************

class RAND_EXT_CLASS Normal : public SymGen              // generate standard normal rv
{
   void operator=(const Normal&) {}       // private so can't access
   static Real Nxi;
   static Real *Nsx;
   static Real *Nsfx;          // so we need initialise only once
   static long count;                     // assume initialised to 0

public:
   char* Name();                          // identification
   Normal();
   ~Normal();
   Real Density(Real) const;              // normal density function
   ExtReal Mean() const { return 0.0; }
   ExtReal Variance() const { return 1.0; }
};

//*********** chi-square random number generator **********************

class RAND_EXT_CLASS ChiSq : public MyRandom               // generate non-central chi-sq rv
{
   void operator=(const ChiSq&) {}        // private so can't access
   MyRandom* c1;                            // pointers to generators
   MyRandom* c2;                            // pointers to generators
   int version;                           // indicates method of generation
   Real mean, var;

public:
   char* Name();                          // identification
   ChiSq(int, Real=0.0);                  // df and non-centrality parameter
   ~ChiSq();
   ExtReal Mean() const { return mean; }
   ExtReal Variance() const { return var; }
   Real Next();
};

//**************** Cauchy random number generator **********************

class RAND_EXT_CLASS Cauchy : public SymGen              // generate standard cauchy rv
{
   void operator=(const Cauchy&) {}       // private so can't access

public:
   char* Name();                          // identification
   Real Density(Real) const;              // Cauchy density function
   ExtReal Mean() const { return Indefinite; }
   ExtReal Variance() const { return PlusInfinity; }
};

//**************** Exponential random number generator **********************

class RAND_EXT_CLASS Exponential : public PosGen         // generate standard exponential rv
{
   void operator=(const Exponential&) {}  // private so can't access
protected:
	Real beta;
public:
	Exponential() {beta = 1.0;}
   Exponential(Real);
   char* Name();                          // identification
   Real Density(Real) const;              // Exponential density function
   ExtReal Mean() const { return 1.0/beta; }
   ExtReal Variance() const { return 1.0/beta/beta; }
};

//**************** asymmetric random number generator **********************

class RAND_EXT_CLASS AsymGen : public MyRandom             // generate asymmetric rv
{
   void operator=(const AsymGen&) {}      // private so can't access
   Real xi, *sx, *sfx; int ic;
   bool NotReady;
   void Build();                          // called on first call to Next

protected:
   Real mode;

public:
   char* Name();                          // identification
   AsymGen(Real);                         // constructor (Real=mode)
   ~AsymGen();                            // destructor
   Real Next();                           // to get a single new value
   ExtReal Mean() const { return (ExtReal)Missing; }
   ExtReal Variance() const { return (ExtReal)Missing; }
};

//**************** Gamma random number generator **********************

class RAND_EXT_CLASS Gamma : public MyRandom               // generate gamma rv
{
   void operator=(const Gamma&) {}        // private so can't access
   MyRandom* method;

public:
   char* Name();                          // identification
   Gamma(Real, Real);                           // constructor (Real=shape)
   Gamma(Real);
   ~Gamma() { delete method; }
   Real Next() { return method->Next(); }
   ExtReal Mean() const { return method->Mean(); }
   ExtReal Variance() const { return method->Variance(); }
};

//**************** Generators with pointers to pdf ***********************

class RAND_EXT_CLASS PosGenX : public PosGen
{
   void operator=(const PosGenX&) {}      // private so can't access
   PDF f;

public:
   char* Name();                          // identification
   PosGenX(PDF fx);
   Real Density(Real x) const { return (*f)(x); }
};

class RAND_EXT_CLASS SymGenX : public SymGen
{
   void operator=(const SymGenX&) {}      // private so can't access
   PDF f;

public:
   char* Name();                          // identification
   SymGenX(PDF fx);
   Real Density(Real x) const { return (*f)(x); }
};

class RAND_EXT_CLASS AsymGenX : public AsymGen
{
   void operator=(const AsymGenX&) {}     // private so can't access
   PDF f;

public:
   char* Name();                          // identification
   AsymGenX(PDF fx, Real mx);
   Real Density(Real x) const { return (*f)(x); }
};

//***************** Pareto random number generator ************************

class RAND_EXT_CLASS Pareto : public MyRandom
// Use definition of Kotz and Johnson: "Continuous univariate distributions 1",
// chapter 19 with k = 1.
{
   void operator=(const Pareto&) {}       // private so can't access
   Real Shape, RS;

public:
   char* Name();                          // identification
   Pareto(Real shape);                    // constructor (Real=shape)
   ~Pareto() {}
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
};


//**************** discrete random number generator **********************

class RAND_EXT_CLASS DiscreteGen : public MyRandom         // discrete random generator
{
   void operator=(const DiscreteGen&) {}  // private so can't access
   Real *p; int *ialt; int n; Real *val;
   void Gen(int, Real*);                  // called by constructors
   Real mean, var;                        // calculated by the constructor

public:
   char* Name();                          // identification
   DiscreteGen(int,Real*);                // constructor
   DiscreteGen(int,Real*,Real*);          // constructor
   ~DiscreteGen();                        // destructor
   Real Next();                           // new single value
   ExtReal Mean() const { return mean; }
   ExtReal Variance() const { return var; }
};

//***************** Poisson random number generator *******************

class RAND_EXT_CLASS Poisson : public MyRandom             // generate poisson rv
{
   void operator=(const Poisson&) {}      // private so can't access
   MyRandom* method;

public:
   char* Name();                          // identification
   Poisson(Real);                         // constructor (Real=mean)
   ~Poisson() { delete method; }
   Real Next() { return method->Next(); }
   ExtReal Mean() const { return method->Mean(); }
   ExtReal Variance() const { return method->Variance(); }
};

//***************** binomial random number generator *******************

class RAND_EXT_CLASS Binomial : public MyRandom            // generate binomial rv
{
   void operator=(const Binomial&) {}     // private so can't access
   MyRandom* method;

public:
   char* Name();                          // identification
   Binomial(int p, Real n);               // constructor (int=n, Real=p)
   ~Binomial() { delete method; }
   Real Next() { return method->Next(); }
   ExtReal Mean() const { return method->Mean(); }
   ExtReal Variance() const { return method->Variance(); }
};

//************** negative binomial random number generator *****************

class RAND_EXT_CLASS NegativeBinomial : public AsymGen   // generate negative binomial rv
{
   Real N, P, Q, p, ln_q, c;

public:
   char* Name();
   NegativeBinomial(Real NX, Real PX);    // constructor
   Real Density(Real) const;              // Negative binomial density function
   Real Next();
   ExtReal Mean() const { return N * P; }
   ExtReal Variance() const { return N * P * Q; }
};



//************************ sum of random numbers ************************

class RAND_EXT_CLASS NegatedRandom : public MyRandom
{
protected:
   MyRandom* rv;
   NegatedRandom(MyRandom& rvx) : rv(&rvx) {}
   void tDelete() { rv->tDelete(); delete this; }

public:
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
   RAND_EXT_CLASS friend NegatedRandom& operator-(MyRandom&);
};

class RAND_EXT_CLASS ScaledRandom : public MyRandom
{
protected:
   MyRandom* rv; Real s;
   ScaledRandom(MyRandom& rvx, Real sx) : rv(&rvx), s(sx) {}
   void tDelete() { rv->tDelete(); delete this; }

public:
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
   RAND_EXT_CLASS friend ScaledRandom& operator*(Real, MyRandom&);
   RAND_EXT_CLASS friend ScaledRandom& operator*(MyRandom&, Real);
   RAND_EXT_CLASS friend ScaledRandom& operator/(MyRandom&, Real);
};

class RAND_EXT_CLASS ReciprocalRandom : public MyRandom
{
protected:
   MyRandom* rv; Real s;
   ReciprocalRandom(MyRandom& rvx, Real sx) : rv(&rvx), s(sx) {}
   void tDelete() { rv->tDelete(); delete this; }

public:
   Real Next();
   ExtReal Mean() const { return Missing; }
   ExtReal Variance() const { return Missing; }
   RAND_EXT_CLASS friend ReciprocalRandom& operator/(Real, MyRandom&);
};

class RAND_EXT_CLASS ShiftedRandom : public ScaledRandom
{
   ShiftedRandom(MyRandom& rvx, Real sx) : ScaledRandom(rvx, sx) {}

public:
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
   RAND_EXT_CLASS friend ShiftedRandom& operator+(Real, MyRandom&);
   RAND_EXT_CLASS friend ShiftedRandom& operator+(MyRandom&, Real);
   RAND_EXT_CLASS friend ShiftedRandom& operator-(MyRandom&, Real);
};

class RAND_EXT_CLASS ReverseShiftedRandom : public ScaledRandom
{
   ReverseShiftedRandom(MyRandom& rvx, Real sx) : ScaledRandom(rvx, sx) {}

public:
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
   RAND_EXT_CLASS friend ReverseShiftedRandom& operator-(Real, MyRandom&);
};

class RAND_EXT_CLASS RepeatedRandom : public MyRandom
{
   MyRandom* rv; int n;
   void tDelete() { rv->tDelete(); delete this; }
   RepeatedRandom(MyRandom& rvx, int nx)  : rv(&rvx), n(nx) {}

public:
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
   friend class MyRandom;
};

class RAND_EXT_CLASS MultipliedRandom : public MyRandom
{
protected:
   MyRandom* rv1; MyRandom* rv2;
   void tDelete() { rv1->tDelete(); rv2->tDelete(); delete this; }
   MultipliedRandom(MyRandom& rv1x, MyRandom& rv2x) : rv1(&rv1x), rv2(&rv2x) {}

public:
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
   RAND_EXT_CLASS friend MultipliedRandom& operator*(MyRandom&, MyRandom&);
};

class RAND_EXT_CLASS AddedRandom : public MultipliedRandom
{
   AddedRandom(MyRandom& rv1x, MyRandom& rv2x) : MultipliedRandom(rv1x, rv2x) {}

public:
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
   RAND_EXT_CLASS friend AddedRandom& operator+(MyRandom&, MyRandom&);
};

class RAND_EXT_CLASS SubtractedRandom : public MultipliedRandom
{
   SubtractedRandom(MyRandom& rv1x, MyRandom& rv2x)
      : MultipliedRandom(rv1x, rv2x) {}

public:
   Real Next();
   ExtReal Mean() const;
   ExtReal Variance() const;
   RAND_EXT_CLASS friend SubtractedRandom& operator-(MyRandom&, MyRandom&);
};

class RAND_EXT_CLASS DividedRandom : public MultipliedRandom
{
   DividedRandom(MyRandom& rv1x, MyRandom& rv2x)
      : MultipliedRandom(rv1x, rv2x) {}

public:
   Real Next();
   ExtReal Mean() const { return Missing; }
   ExtReal Variance() const { return Missing; }
   RAND_EXT_CLASS friend DividedRandom& operator/(MyRandom&, MyRandom&);
};

class RAND_EXT_CLASS SumRandom : public MyRandom           // sum of random variables
{
   void operator=(const SumRandom&) {}    // private so can't access
   MyRandom* rv;

public:
   char* Name();                          // identification
   SumRandom(NegatedRandom& rvx) : rv(&rvx) {}
   SumRandom(AddedRandom& rvx) : rv(&rvx) {}
   SumRandom(MultipliedRandom& rvx) : rv(&rvx) {}
   SumRandom(SubtractedRandom& rvx) : rv(&rvx) {}
   SumRandom(DividedRandom& rvx) : rv(&rvx) {}
   SumRandom(ShiftedRandom& rvx) : rv(&rvx) {}
   SumRandom(ReverseShiftedRandom& rvx) : rv(&rvx) {}
   SumRandom(ScaledRandom& rvx) : rv(&rvx) {}
   SumRandom(ReciprocalRandom& rvx) : rv(&rvx) {}
   SumRandom(RepeatedRandom& rvx) : rv(&rvx) {}
   Real Next() { return rv->Next(); }
   ExtReal Mean() const { return rv->Mean(); }
   ExtReal Variance() const { return rv->Variance(); }
   ~SumRandom() { rv->tDelete(); }
};

//******************** mixtures of random numbers ************************


class RAND_EXT_CLASS SelectedRandom : public MyRandom
{
   friend class MyRandom;
   MyRandom* rv; Real p;
   SelectedRandom(MyRandom& rvx, Real px) : rv(&rvx), p(px) {}
   void tDelete() { rv->tDelete(); delete this; }

public:
   void load(int*, Real*, MyRandom**);
   Real Next();
};

class RAND_EXT_CLASS AddedSelectedRandom : public MyRandom
{
   friend class MyRandom;

protected:
   MyRandom* rv1; MyRandom* rv2;
   AddedSelectedRandom(MyRandom& rv1x, MyRandom& rv2x)
      : rv1(&rv1x), rv2(&rv2x) {}
   void tDelete() { rv1->tDelete(); rv2->tDelete(); delete this; }

public:
   int nelems() const;
   void load(int*, Real*, MyRandom**);
   Real Next();
   RAND_EXT_CLASS friend AddedSelectedRandom& operator+(SelectedRandom&,
      SelectedRandom&);
   RAND_EXT_CLASS friend AddedSelectedRandom& operator+(AddedSelectedRandom&,
      SelectedRandom&);
   RAND_EXT_CLASS friend AddedSelectedRandom& operator+(SelectedRandom&,
      AddedSelectedRandom&);
   RAND_EXT_CLASS friend AddedSelectedRandom& operator+(AddedSelectedRandom&,
      AddedSelectedRandom&);
};

class RAND_EXT_CLASS MixedRandom : public MyRandom         // mixtures of random numbers
{
   void operator=(const MixedRandom&) {}  // private so can't access
   int n;                                 // number of components
   DiscreteGen* dg;                       // discrete mixing distribution
   MyRandom** rv;                           // array of pointers to rvs
   ExtReal mean, var;
   void Build(Real*);                     // used by constructors

public:
   char* Name();                          // identification
   MixedRandom(int, Real*, MyRandom**);
   MixedRandom(AddedSelectedRandom&);
   ~MixedRandom();
   Real Next();
   ExtReal Mean() const { return mean; }
   ExtReal Variance() const { return var; }
};

//******************* Permutation generator *******************************

class RAND_EXT_CLASS RandomPermutation                   // generate permutation of integers
{
   MyRandom U;

public:
   void Next(int N, int M, int p[], int start = 0);
                                          // select permutation of M numbers
                                          // from start, ..., start+N-1
                                          // results to p
   void Next(int N, int p[], int start = 0) { Next(N, N, p, start); }
};

class RAND_EXT_CLASS RandomCombination : public RandomPermutation
                                          // generate combination of integers
                                          // sorted - ascending
{
   void SortAscending(int n, int gm[]);

public:
   void Next(int N, int M, int p[], int start = 0)
      { RandomPermutation::Next(N, M, p, start); SortAscending(M, p); }
   void Next(int N, int p[], int start = 0) { Next(N, N, p, start); }
};

//***************** Generators with variable parameters ********************

class RAND_EXT_CLASS VariPoisson
{
   Uniform U;
   Normal N;
   Poisson P100;
   Poisson P200;
   int iNext_very_small(Real mu);
   int iNext_small(Real mu);
   int iNext_large(Real mu);
public:
   VariPoisson();
   int iNext(Real mu);
};

class RAND_EXT_CLASS VariBinomial
{
   Uniform U;
   Normal N;
   int iNext_very_small(int n, Real p);
   int iNext_small(int n, Real p);
   int iNext_large(int n, Real p);
public:
   VariBinomial();
   int iNext(int n, Real p);
};

class RAND_EXT_CLASS VariLogNormal
{
   Normal N;
public:
   VariLogNormal() {}
   Real Next(Real mean, Real sd);
};



RAND_EXT_CLASS void Histogram(MyRandom* rx, int n);
#ifdef use_namespace
}
#endif

#endif

// body file: newran.cpp



