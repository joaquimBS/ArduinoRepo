#ifndef PROTOTEST_H
#define PROTOTEST_H

#define DIH 0
#define DAH 1

void dih();
void dah();

typedef struct {
	int _A[2] =	{DIH,DAH};
	int _B[4] =	{DAH,DIH,DIH,DIH};
	int _C[5] =	{DAH,DIH,DAH,DIH};
	int _D[3] =	{DAH,DIH,DIH};
	int _E[1] =	{DIH};
	int _F[5] =	{DIH,DIH,DAH,DIH};
	int _G[3] =	{DAH,DAH,DIH};
	int _H[5] =	{DIH,DIH,DIH,DIH};
	int _I[2] =	{DIH,DIH};
	int _J[5] =	{DIH,DAH,DAH,DAH};
	int _K[3] =	{DAH,DIH,DAH};
	int _L[5] =	{DIH,DAH,DIH,DIH};
	int _M[2] =	{DAH,DAH};
	int _N[2] =	{DAH,DIH};
	int _O[3] =	{DAH,DAH,DAH};
	int _P[5] =	{DIH,DAH,DAH,DIH};
	int _Q[5] =	{DAH,DAH,DIH,DAH};
	int _R[3] =	{DIH,DAH,DIH};
	int _S[3] =	{DIH,DIH,DIH};
	int _T[1] =	{DAH};
	int _U[3] =	{DIH,DIH,DAH};
	int _V[5] =	{DIH,DIH,DIH,DAH};
	int _W[3] =	{DIH,DAH,DAH};
	int _X[5] =	{DAH,DIH,DIH,DAH};
	int _Y[5] =	{DAH,DIH,DAH,DAH};
	int _Z[5] =	{DAH,DAH,DIH,DIH};
} MorseAlphabet;

char strText[] = "HOLA ZAIRA";

#endif