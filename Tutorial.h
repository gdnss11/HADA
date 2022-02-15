/*----------------------------------------------------------------\
@ Numerical Methods by Young-Keun Kim - Handong Global University

Author           : ChangMin An
Created          : 15-04-2021
Modified         : 15-04-2021
Language/ver     : C++ in MSVS2019

Description      : myNM.h
----------------------------------------------------------------*/

#ifndef		_MY_NM_H		// use either (#pragma once) or  (#ifndef ...#endif)
#define		_MY_NM_H

#include "myMatrix.h"
#include <math.h>

/*===================================
                  Macros
=====================================*/
#define NEW_LINE    printf("\n")
#define SEP_LINE    printf("************************************\n")

// Apply back-substitution
void backSub(Matrix _U, Matrix _d, Matrix _x);

// Apply forward-substitution
void fwdSub(Matrix _L, Matrix _b, Matrix _y);

// Gauss Elimination
void gaussElim(Matrix _A, Matrix _b, Matrix _U, Matrix _d);
void gaussJordanElim(Matrix _A, Matrix _b, Matrix _U, Matrix _d);

//Non-Linear Method
double bisectionNL(double _a0, double _b0, double _tol);
double func(double _x);
double dfunc(double _x);
double newtonRaphson(double _x0, double _tol);
void PrintResult(int _k, double _x, double _tol);
double Hybrid_Func(double _x);
double Hybrid_Dfunc(double _x);
double Hybrid(double* _a0, double* _b0, double _x, double _tol);
double Hybrid_newtonRaphson(double _x0, int* _num, double* eps);
double Hybrid_bisectionNL(double* _a0, double* _b0, int* _num, double* eps);
double Test_newtonRaphson(double _x0, double _tol);

// Define LUdecomp
void LUdecomp(Matrix _A, Matrix _L, Matrix _U, Matrix _P);

// Devine solveLU
void solveLU(Matrix _L, Matrix _U, Matrix _P, Matrix _b, Matrix _x);

void inv(Matrix _A, Matrix Ainv);

//Norm
double	Norm(Matrix _A);

//QR Decomposition
void QRDecomp(Matrix _A, Matrix _Q, Matrix _R);

//Make into similar matrix
void Eig(Matrix _A, Matrix _EigenValue);

//Jacobian
void fzeroNewton(Matrix _xinput, Matrix _xoutput);
void funcVec(Matrix _xinput, Matrix _func);
void funcJac(Matrix _xinput, Matrix _J);

//Condition Number
void Cond(Matrix _A, double* _Out);


/*===========Curve Fitting==========*/
//Linear Curve Fitting
Matrix	linearFit(Matrix _x, Matrix _y);
double  linearFit_Solution(Matrix _z, double _x);

//Linear Spline Interpolation
Matrix	linearInterp(Matrix _x, Matrix _y, Matrix _xq);

/*======================================
         Numerical Differentiation
========================================*/

// Return the dy/dx results for the input data. (truncation error: O(h^2))
Matrix	gradient(Matrix _x, Matrix _y);

// Return the dy/dx results for the input data about 1D array (truncation error: 0(h^2))
void  gradient1D(double _x[], double _y[], double _dydx[], int _m);

// Return the dy/dx results for the target equation. (truncation error: O(h^2))
Matrix	gradientFunc(double func(const double x), Matrix xin);

// Return Non-Linear Solution Using NewtonRaphson Method. (truncation error: 0(h^2))
double newtonRaphsonFunc(double func(const double x), double dfunc(const double x), double x0, double tol);

double trapz(double x[], double y[], int m);

double integral(double func(const double x), double a, double b, int n);

double integralMid(double x[], double y[], int m);

double integral38(double func(const double x), double a, double b, int n);
#endif